#include <Arduino.h>
#include <STM32FreeRTOS.h>
/*===== HARDEWARE =====*/
#include <hardware_cfg.h>
#include <motors.h>
#include <bsp.h>
//IMU
#include <ImuLib_cfg.h>
//PIXEL
#include <PixelLedLib_cfg.h>
/*===== CONNECTIVITY =====*/
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <UartLib.h>
/*===== MICRO ROS =====*/
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
/*===== ROS MSGS TYPES =====*/
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int64.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/joint_state.h>

/* DEFINES */
#define CLIENT_IP "192.168.1.177"
#define AGENT_IP "192.168.1.176"
#define AGENT_PORT 8888
#define NODE_NAME "stm32_node"


#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
      Serial.printf("o");          \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      Serial.printf("!");          \
    }                              \
  }


extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);


typedef struct {
  double lin_x;
  double lin_y;
  double ang_z;
} cmd_vel_queue_t;


typedef struct {
  double pos_x;
  double pos_y;
  double rot_p_z;
  double lin_x;
  double lin_y;
  double rot_v_z;
} odometry_queue_t;

typedef struct {
  uint8_t size = 4;
  double velocity[4];
  double positon[4];
} motor_state_queue_t;


/* VARIABLES */
QueueHandle_t CmdVelQueue;
QueueHandle_t OdomQueue;
QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;

//ROS PUBLISHERS
rcl_publisher_t publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t motor_state_publisher;
//ROS SUBSCRIPTIONS
rcl_subscription_t subscriber;
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t motors_cmd_subscriber;
//ROS MESSAGES
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist* twist_msg;
std_msgs__msg__String msgs;
uint8_t ros_msgs_cnt = 0;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState motors_cmd_msg;
sensor_msgs__msg__JointState motors_response;

//ROS
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//ETHERNET
IPAddress client_ip;
IPAddress agent_ip;
byte mac[] = {0x02, 0x47, 0x00, 0x00, 0x00, 0x01};

//MOTORS
MotorClass Motor1(M1_PWM_PIN, M1_PWM_TIM, M1_PWM_TIM_CH, M1_ILIM, M1A_IN, M1B_IN, M1_ENC_TIM, M1_ENC_A, M1_ENC_B, M1_DEFAULT_DIR);
MotorClass Motor2(M2_PWM_PIN, M2_PWM_TIM, M2_PWM_TIM_CH, M2_ILIM, M2A_IN, M2B_IN, M2_ENC_TIM, M2_ENC_A, M2_ENC_B, M2_DEFAULT_DIR);
MotorClass Motor3(M3_PWM_PIN, M3_PWM_TIM, M3_PWM_TIM_CH, M3_ILIM, M3A_IN, M3B_IN, M3_ENC_TIM, M3_ENC_A, M3_ENC_B, M3_DEFAULT_DIR);
MotorClass Motor4(M4_PWM_PIN, M4_PWM_TIM, M4_PWM_TIM_CH, M4_ILIM, M4A_IN, M4B_IN, M4_ENC_TIM, M4_ENC_A, M4_ENC_B, M4_DEFAULT_DIR);
MotorPidClass M1_PID(&Motor1);
MotorPidClass M2_PID(&Motor2);
MotorPidClass M3_PID(&Motor3);
MotorPidClass M4_PID(&Motor4);

//IMU
extern ImuDriver ImuBno;
//PIXEL LED
PixelLedClass PixelStrip(PIXEL_LENGTH, VIRTUAL_LED_LENGTH, 0);

/* TASKS DECLARATION */
static void rclc_spin_task(void *p);
static void imu_task(void *p);
static void chatter_publisher_task(void *p);
static void runtime_stats_task(void *p);
static void pid_handler_task(void *p);
static void pixel_led_task(void *p);
static void kinematics_task(void *p);
static void board_support_task(void *p);
static void power_board_task(void *p);

/* FUNCTIONS */
void error_loop() {
  while (1) {
    // Serial.printf("in error loop");
    SetRedLed(Toggle);
    delay(100);
  }
}

void motors_cmd_callback(const void *msgin){
  static double Setpoint[] = {0,0,0,0};
  static sensor_msgs__msg__JointState * setpoint_msg;
  setpoint_msg = (sensor_msgs__msg__JointState *)msgin;
  // char* joint_name = (char*)setpoint_msg->name.data;
  // if(joint_name == "RR")  Setpoint[0] = *((double*)setpoint_msg->velocity.data);
  // if(joint_name == "RL")  Setpoint[1] = *((double*)setpoint_msg->velocity.data);
  // if(joint_name == "FR")  Setpoint[2] = *((double*)setpoint_msg->velocity.data);
  // if(joint_name == "FL")  Setpoint[3] = *((double*)setpoint_msg->velocity.data);
  // Setpoint[0] = *((double*)setpoint_msg->velocity.data);
  // Setpoint[1] = *((double*)setpoint_msg->velocity.data);
  // Setpoint[2] = *((double*)setpoint_msg->velocity.data);
  Setpoint[0] = (double)setpoint_msg->velocity.data[0];
  Setpoint[1] = (double)setpoint_msg->velocity.data[1];
  Setpoint[2] = (double)setpoint_msg->velocity.data[2];
  Setpoint[3] = (double)setpoint_msg->velocity.data[3];
  // motors_response.velocity.size = motor_state_queue.size;
  // motors_response.velocity.data = motor_state_queue.velocity;
  xQueueSendToFront(SetpointQueue, (void*) Setpoint, (TickType_t) 0);
}

void cmd_vel_callback(const void *msgin){
  static cmd_vel_queue_t queue_cmd;
  static geometry_msgs__msg__Twist * twist_msg;
  twist_msg = (geometry_msgs__msg__Twist *)msgin;
  queue_cmd.lin_x = twist_msg->linear.x;
  queue_cmd.lin_y = twist_msg->linear.y;
  queue_cmd.ang_z = twist_msg->angular.z;
  xQueueSendToFront(CmdVelQueue, (void*) &queue_cmd, (TickType_t) 0);
}



const void euler_to_quat(float x, float y, float z, double* q) {
    float c1 = cos((y)/2);
    float c2 = cos((z)/2);
    float c3 = cos((x)/2);

    float s1 = sin((y)/2);
    float s2 = sin((z)/2);
    float s3 = sin((x)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}



void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  static odometry_queue_t queue_odom;
  static imu_queue_t queue_imu;
  static motor_state_queue_t motor_state_queue;
  if (timer != NULL) {
    if (xQueueReceive(OdomQueue, &queue_odom, (TickType_t) 0) == pdPASS) {
      struct timespec ts = {0};
      clock_gettime(CLOCK_REALTIME, &ts);

      odom_msg.header.stamp.sec = ts.tv_sec;
      odom_msg.header.stamp.nanosec = ts.tv_nsec;

      odom_msg.header.frame_id.data = (char *) "odom";
      odom_msg.child_frame_id.data = (char *) "base_link";

      odom_msg.pose.pose.position.x = queue_odom.pos_x;
      odom_msg.pose.pose.position.y = queue_odom.pos_y;

      double quat[4];
      euler_to_quat(0.0, 0.0, queue_odom.rot_p_z, quat);
      odom_msg.pose.pose.orientation.w = quat[0];
      odom_msg.pose.pose.orientation.x = quat[1];
      odom_msg.pose.pose.orientation.y = quat[2];
      odom_msg.pose.pose.orientation.z = quat[3];

      odom_msg.twist.twist.linear.x = queue_odom.lin_x;
      odom_msg.twist.twist.linear.y = queue_odom.lin_y;
      odom_msg.twist.twist.angular.z = queue_odom.rot_v_z;

      RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    }
    if(xQueueReceive(ImuQueue, &queue_imu, (TickType_t) 0) == pdPASS){
      struct timespec ts = {0};
      clock_gettime(CLOCK_REALTIME, &ts);
      imu_msg.header.stamp.sec = ts.tv_sec;
      imu_msg.header.stamp.nanosec = ts.tv_nsec;
      imu_msg.header.frame_id.data = (char *) "imu";
      imu_msg.orientation.x = queue_imu.Orientation[0];
      imu_msg.orientation.y = queue_imu.Orientation[1];
      imu_msg.orientation.z = queue_imu.Orientation[2];
      imu_msg.orientation.w = queue_imu.Orientation[3];
      imu_msg.angular_velocity.x = queue_imu.AngularVelocity[0];
      imu_msg.angular_velocity.y = queue_imu.AngularVelocity[1];
      imu_msg.angular_velocity.z = queue_imu.AngularVelocity[2];
      imu_msg.linear_acceleration.x = queue_imu.LinearAcceleration[0];
      imu_msg.linear_acceleration.y = queue_imu.LinearAcceleration[1];
      imu_msg.linear_acceleration.z = queue_imu.LinearAcceleration[2];
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
    if(xQueueReceive(MotorStateQueue, &motor_state_queue, (TickType_t) 0) == pdPASS){
      struct timespec ts = {0};
      clock_gettime(CLOCK_REALTIME, &ts);
      motors_response.header.stamp.sec = ts.tv_sec;
      motors_response.header.stamp.nanosec = ts.tv_nsec;
      motors_response.header.frame_id.data = (char*) "motors_response";
      motors_response.velocity.size = motor_state_queue.size;
      motors_response.velocity.data = motor_state_queue.velocity;
      RCSOFTCHECK(rcl_publish(&motor_state_publisher, &motors_response, NULL));
    }
  }
}

void init_odom_msg() {
  odom_msg.header.frame_id.data = (char *) "odom";
  odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

  odom_msg.child_frame_id.data = (char *) "base_link";
  odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

  odom_msg.pose.covariance[0] = 0.01;
  odom_msg.pose.covariance[6] = 0.01;
  odom_msg.pose.covariance[35] = 0.01;

  odom_msg.twist.covariance[0] = 0.01;
  odom_msg.twist.covariance[6] = 0.01;
  odom_msg.twist.covariance[35] = 0.01;
}


/*==================== SETUP ========================*/
void setup() {
  //Hardware init
  BoardGpioInit();
  BoardPheripheralsInit();
  SetLocalPower(On);
  delay(1000);
  portBASE_TYPE s1, s2, s3, s4, s5, s6, s7, s8, s9;
  //Motors init
  M1_PID.SetSetpoint(0);
  M2_PID.SetSetpoint(0);
  M3_PID.SetSetpoint(0);
  M4_PID.SetSetpoint(0);

  //Pixel Led
  PixelStrip.Init();
  ImuBno.Init();

  delay(2000);

  client_ip.fromString(CLIENT_IP);
  agent_ip.fromString(AGENT_IP);

  if(BOARD_MODE_DEBUG){
    Serial.printf("Connecting to agent: \r\n");
    Serial.println(agent_ip);
  }


  set_microros_native_ethernet_udp_transports(mac, client_ip, agent_ip,
                                              AGENT_PORT);
  delay(2000);

  while(rmw_uros_ping_agent(50, 2) != RMW_RET_OK) {
    SetRedLed(Toggle);
    delay(100);
    continue;
  }

  allocator = rcl_get_default_allocator();

  /* MICRO ROS INIT */
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))
  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  // init timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50),
                                  timer_callback));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created timer\r\n");
  // Init subscribers
  RCCHECK(rclc_subscription_init_default(
      &motors_cmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "motors_cmd"));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created 'motors_cmd' subscriber\r\n");

  // RCCHECK(rclc_subscription_init_default(
  //     &cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  //     "cmd_vel"));
  // ros_msgs_cnt++;
  // if(BOARD_MODE_DEBUG) Serial.printf("Created 'cmd_vel' subscriber\r\n");
  // Init publishers
  // RCCHECK(rclc_publisher_init_default(
  //     &odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  //     "odom/wheels"));
  // ros_msgs_cnt++;
  // if(BOARD_MODE_DEBUG) Serial.printf("Created 'odom/wheels' publisher.\r\n");

  RCCHECK(rclc_publisher_init_default(
      &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data_raw"));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created 'sensor_msgs/Imu' publisher.\r\n");

  RCCHECK(rclc_publisher_init_default(
      &motor_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "motors_response"));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created 'motors_response' publisher.\r\n");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, ros_msgs_cnt, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &msgs,
  //                                       &cmd_vel_callback, ON_NEW_DATA));   
  RCCHECK(rclc_executor_add_subscription(&executor, &motors_cmd_subscriber, &motors_cmd_msg,
                                        &motors_cmd_callback, ON_NEW_DATA));                                 
  if(BOARD_MODE_DEBUG) Serial.printf("Executor started\r\n");
  RCCHECK(rmw_uros_sync_session(1000));
  if(BOARD_MODE_DEBUG) Serial.printf("Clocks synchronised\r\n");

  /* RTOS QUEUES CREATION */
  CmdVelQueue = xQueueCreate(1, sizeof(cmd_vel_queue_t));
  OdomQueue = xQueueCreate(1, sizeof(odometry_queue_t));
  SetpointQueue = xQueueCreate(1, sizeof(double)*4);
  // MotorStateQueue = xQueueCreate(1, sizeof(double)*4);
  MotorStateQueue = xQueueCreate(1, sizeof(motor_state_queue_t));
  ImuQueue = xQueueCreate(1, sizeof(imu_queue_t));
  if(BOARD_MODE_DEBUG) Serial.printf("Queues created\r\n");

  /* MICRO ROS MESSAGES INIT */
  init_odom_msg();

  /* RTOS TASKS CREATION */
  s1 = xTaskCreate(rclc_spin_task, "rclc_spin_task",
                   configMINIMAL_STACK_SIZE + 7000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if(s1 != pdPASS)  Serial.printf("S1 creation problem\r\n");
  s2 = xTaskCreate(imu_task, "imu_task",
                   configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if(s2 != pdPASS)  Serial.printf("S2 creation problem\r\n");
  s3 = xTaskCreate(runtime_stats_task, "runtime_stats_task",
                   configMINIMAL_STACK_SIZE + 1500, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if(s3 != pdPASS)  Serial.printf("S3 creation problem\r\n");
  s4 = xTaskCreate(pid_handler_task, "pid_handler_task",
                            configMINIMAL_STACK_SIZE + 800, NULL, tskIDLE_PRIORITY + 2,
                            NULL);
  // if(s4 != pdPASS)  Serial.printf("S4 creation problem\r\n");
  // s5 = xTaskCreate(pixel_led_task, "pixel_led_task",
  //                         configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
  //                         NULL);
  // if(s5 != pdPASS)  Serial.printf("S5 creation problem\r\n");
  // s6 = xTaskCreate(kinematics_task, "kinematics_task",
  //                         configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
  //                         NULL);
  // if(s6 != pdPASS)  Serial.printf("S6 creation problem\r\n");
  // s7 = xTaskCreate(board_support_task, "board_support_task",
  //                         configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
  //                         NULL);
  // if(s7 != pdPASS)  Serial.printf("S7 creation problem\r\n");
  // s8 = xTaskCreate(power_board_task, "power_board_task",
  //                         configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
  //                         NULL);
  // if(s8 != pdPASS)  Serial.printf("S8 creation problem\r\n");

  /* HARDWARE ACTIONS BEFORE RTOS STARTING */

//Allocate memory for motors response message
  motors_response.name.capacity = 4;
  motors_response.name.size = 4;
  motors_response.name.data = (rosidl_runtime_c__String*) malloc(motors_response.name.capacity*sizeof(rosidl_runtime_c__String));
  for(int i=0;i<4;i++) {
      motors_response.name.data[i].data = (char*)malloc(2);
      motors_response.name.data[i].capacity = 2;
      sprintf(motors_response.name.data[i].data,"M%d",i+1);
      motors_response.name.data[i].size = strlen(motors_response.name.data[i].data);
  }
  motors_response.velocity.size=4;
  motors_response.velocity.capacity=4;
  motors_response.velocity.data = (double*)malloc(motors_response.velocity.capacity*sizeof(double));

  motors_cmd_msg.name.capacity = 4;
  motors_cmd_msg.name.size = 4;
  motors_cmd_msg.name.data = (rosidl_runtime_c__String*) malloc(motors_cmd_msg.name.capacity*sizeof(rosidl_runtime_c__String));
  for(int i=0;i<4;i++) {
      motors_cmd_msg.name.data[i].data = (char*)malloc(2);
      motors_cmd_msg.name.data[i].capacity = 2;
      sprintf(motors_cmd_msg.name.data[i].data,"M%d",i+1);
      motors_cmd_msg.name.data[i].size = strlen(motors_cmd_msg.name.data[i].data);
  }
  motors_cmd_msg.velocity.size=4;
  motors_cmd_msg.velocity.capacity=4;
  motors_cmd_msg.velocity.data = (double*)malloc(motors_cmd_msg.velocity.capacity*sizeof(double));



  SetGreenLed(On);

  /* START RTOS */
  if(BOARD_MODE_DEBUG) Serial.printf("Tasks starting\r\n");
  vTaskStartScheduler();
}

static void rclc_spin_task(void *p) {
  UNUSED(p);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    RCSOFTCHECK(rclc_executor_spin(&executor));
    vTaskDelayUntil(&xLastWakeTime, 2);
  }
}

static void imu_task(void *p){
  static imu_queue_t queue_imu;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    queue_imu = ImuBno.LoopHandler();
    xQueueSendToFront(ImuQueue, (void*) &queue_imu, TickType_t(0));
    // vTaskDelay(1000/IMU_SAMPLE_FREQ*portTICK_PERIOD_MS);
    vTaskDelayUntil(&xLastWakeTime, (1000/IMU_SAMPLE_FREQ*portTICK_PERIOD_MS));
  }
}

static void runtime_stats_task(void *p) {
  char buf[2000];
  Serial.printf("runtime stats task started\r\n");
  while (1) {
    vTaskGetRunTimeStats(buf);
    Serial.printf("\r\n%s\r\n-------------", buf);
    vTaskDelay(2000);
  }
}


static void pid_handler_task(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();

  double setpoint[4];
  // double motor_state[4];
  static motor_state_queue_t motor_state;

  while(1){
    vTaskDelayUntil(&xLastWakeTime, (1000/PID_FREQ*portTICK_PERIOD_MS));
    xQueueReceive(SetpointQueue, &setpoint, (TickType_t) 0);

    M4_PID.Handler();
    M4_PID.SetSetpoint(setpoint[3]);
    // motor_state[3] = double(M4_PID.Motor->GetVelocity());
    motor_state.velocity[3] = (double)M4_PID.Motor->GetVelocity();

    M2_PID.Handler();
    M2_PID.SetSetpoint(setpoint[1]);
    // motor_state[1] = double(M2_PID.Motor->GetVelocity());
    motor_state.velocity[1] = (double)M2_PID.Motor->GetVelocity();

    M1_PID.Handler();
    M1_PID.SetSetpoint(setpoint[0]);
    // motor_state[0] = double(M1_PID.Motor->GetVelocity());
    motor_state.velocity[0] = (double)M1_PID.Motor->GetVelocity();

    M3_PID.Handler();
    M3_PID.SetSetpoint(setpoint[2]);
    // motor_state[2] = double(M3_PID.Motor->GetVelocity());
    motor_state.velocity[2] = (double)M3_PID.Motor->GetVelocity();

    xQueueSendToFront(MotorStateQueue, (void*) &motor_state, (TickType_t) 0);
  }
}


static void pixel_led_task(void *p){
  uint16_t DelayTime = 2000;
  while(1){
    vTaskDelay(DelayTime);
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x0F, 0x0F, 0x0F, 50);
    vTaskDelay(DelayTime);
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x00, 0x00, 0x0F, 50);
  }
}

static void kinematics_task(void *p) {
  static cmd_vel_queue_t queue_cmd;
  static odometry_queue_t queue_odom;
  static int64_t motor_vel_handler = 0;

  static double setpoint[4];
  double motor_state[4];

  double setpoint_right = 0.0;
  double setpoint_left = 0.0;
  static uint32_t LastTime;
  static uint32_t ActualTime;
  double sleep_time;
  LastTime = ActualTime = xTaskGetTickCount();
  vTaskDelay(40);

  // RR, RL, FR, FL

  while(1) {
    // Get currently set velocity
    xQueueReceive(CmdVelQueue, (void*) &queue_cmd, (TickType_t) 0);


    setpoint_right =  (1/(WHEEL_DIAM/2)) * (queue_cmd.lin_x + (ROBOT_WIDTH + ROBOT_LENGTH) * queue_cmd.ang_z);
    setpoint_left =   (1/(WHEEL_DIAM/2)) * (queue_cmd.lin_x - (ROBOT_WIDTH + ROBOT_LENGTH) * queue_cmd.ang_z);

    setpoint[0] = setpoint_right;
    setpoint[1] = setpoint_left;
    setpoint[2] = setpoint_right;
    setpoint[3] = setpoint_left;

    // Update motor setpoints
    // xQueueSendToFront(SetpointQueue, (void*) setpoint, (TickType_t) 0);


    // Read motor positions
    // xQueueReceive(MotorStateQueue, (void*) motor_state, (TickType_t) 0);
    

    // if(BOARD_MODE_DEBUG) Serial.println(motor_state[0].pos);
    queue_odom.lin_x = ((motor_state[0] + motor_state[1] + motor_state[2] + motor_state[3])/4)*WHEEL_DIAM/2;
    queue_odom.rot_v_z = (-motor_state[3] + motor_state[2] - motor_state[1] + motor_state[0]) * (
                          WHEEL_DIAM/2/(4 * (ROBOT_WIDTH / 2 + ROBOT_LENGTH / 2)));
    // queue_odom.lin_y = (-motor_state[3] + motor_state[2] + motor_state[1] - motor_state[0]) * (wheel_diameter/4);

    ActualTime = xTaskGetTickCount();
    sleep_time = double(ActualTime - LastTime) / 1000;
    queue_odom.rot_p_z += queue_odom.rot_v_z * sleep_time;
    // queue_odom.rot_p_z = 0.0; 
    queue_odom.pos_x += (queue_odom.lin_x * cos(queue_odom.rot_p_z) - queue_odom.lin_y * sin(queue_odom.rot_p_z)) * sleep_time;
    queue_odom.pos_y += (queue_odom.lin_x * sin(queue_odom.rot_p_z) + queue_odom.lin_y * cos(queue_odom.rot_p_z)) * sleep_time;

    // Output odometry
    xQueueSendToFront(OdomQueue, &queue_odom, (TickType_t) 0);

    LastTime = ActualTime;
    // vTaskDelay(TickType_t(40 / portTICK_PERIOD_MS));
    vTaskDelay(1000/KINEMATIC_TASK_FREQ*portTICK_PERIOD_MS);
  }
}

static void board_support_task(void *p){
  while(1){
    if(digitalRead(PWR_BRD_GPIO_INPUT)){
      //send information to SBC to turn off
    }
    vTaskDelay(500);
  }
}

static void power_board_task(void *p){
  static String Frame = "Temp_Text";
  static char FrameBuffer[100];
  while(1){
    // Serial2.readStringUntil('<');
    // frame = Serial2.readStringUntil('>');
    // sprintf(FrameBuffer, "Temp");
    // Serial.printf("%c", FrameBuffer);
    vTaskDelay(100);
  }
}


/*============== LOOP - IDDLE TASK ===============*/

void loop() {
  SetGreenLed(Toggle);
  delay(1000);
}

/*=========== Runtime stats ====================*/

HardwareTimer stats_tim(TIM5);  // TIM5 - 32 bit

void vConfigureTimerForRunTimeStats(void) {
  //   m1_pwm.setPWM(1, M1_PWM, 1000, power);
  stats_tim.setPrescaleFactor(
      1680);  // Set prescaler to 2564 => timer frequency = 168MHz/1680 = 100000
              // Hz (from prediv'd by 1 clocksource of 168 MHz)
  stats_tim.setOverflow(0xffffffff);  // Set overflow to 32761 => timer
                                      // frequency = 65522 Hz / 32761 = 2 Hz
  stats_tim.refresh();                // Make register changes take effect
  stats_tim.resume();                 // Start
}

uint32_t vGetTimerValueForRunTimeStats(void) { return stats_tim.getCount(); }
