#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_cfg.h>
/*===== HARDEWARE =====*/
#include <hardware_cfg.h>
#include <bsp.h>
//MOTORS
#include <motors.h>
//IMU
#include <ImuLib_cfg.h>
//PIXEL
#include <PixelLedLib_cfg.h>
/*===== CONNECTIVITY =====*/
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <UartLib.h>

/* VARIABLES */
bool uRosInitSuccesfull = false;
//RTOS
QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;
QueueHandle_t BatteryStateQueue;
portBASE_TYPE s1, s2, s3, s4, s5, s6, s7, s8, s9;

/* EXTERN VARIABLES */
extern UartProtocolClass PowerBoardSerial;
//IMU
extern ImuDriver ImuBno;
//microROS
extern std_msgs__msg__String msgs;
extern sensor_msgs__msg__Imu imu_msg;
extern sensor_msgs__msg__JointState motors_cmd_msg;
extern sensor_msgs__msg__JointState motors_response_msg;
extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t motor_state_publisher;
//MOTORS
extern MotorPidClass M1_PID;
extern MotorPidClass M2_PID;
extern MotorPidClass M3_PID;
extern MotorPidClass M4_PID;
//LED
extern PixelLedClass PixelStrip;

//ETHERNET
IPAddress client_ip;
IPAddress agent_ip;
byte mac[] = {0x02, 0x47, 0x00, 0x00, 0x00, 0x01};

/* RTOS TASKS DECLARATIONS */
static void rclc_spin_task(void *p);
static void imu_task(void *p);
static void runtime_stats_task(void *p);
static void pid_handler_task(void *p);
static void pixel_led_task(void *p);
static void board_support_task(void *p);
static void power_board_task(void *p);
static void motors_response_task(void *p);

/* FUNCTIONS */

void EthernetInit(){
  client_ip.fromString(CLIENT_IP);
  agent_ip.fromString(AGENT_IP);
  if(BOARD_MODE_DEBUG){
    Serial.printf("Connecting to agent: \r\n");
    Serial.println(agent_ip);
  }
  set_microros_native_ethernet_udp_transports(mac, client_ip, agent_ip,
                                              AGENT_PORT);
  delay(1000);
}

/*==================== SETUP ========================*/
void setup() {
  //Hardware init
  BoardGpioInit();
  BoardPheripheralsInit();
  SetLocalPower(On);
  delay(1000);
  
  PixelStrip.Init();
  ImuBno.Init();
  EthernetInit();

  while(uRosPingAgent() != Ok){
    SetRedLed(Toggle);
    delay(100);
  }

  /* RTOS QUEUES CREATION */
  SetpointQueue = xQueueCreate(1, sizeof(double)*4);
  MotorStateQueue = xQueueCreate(1, sizeof(motor_state_queue_t));
  ImuQueue = xQueueCreate(1, sizeof(imu_queue_t));
  BatteryStateQueue = xQueueCreate(1, sizeof(battery_state_queue_t));
  if(BOARD_MODE_DEBUG) Serial.printf("Queues created\r\n");
  // uRosInitSuccesfull = uRosCreateEntities();
  /* RTOS TASKS CREATION */
  s1 = xTaskCreate(rclc_spin_task, "rclc_spin_task",
                   configMINIMAL_STACK_SIZE + 3000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if(s1 != pdPASS)  Serial.printf("S1 creation problem\r\n");
  s2 = xTaskCreate(imu_task, "imu_task",
                   configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if(s2 != pdPASS)  Serial.printf("S2 creation problem\r\n");
  s3 = xTaskCreate(runtime_stats_task, "runtime_stats_task",
                   configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if(s3 != pdPASS)  Serial.printf("S3 creation problem\r\n");
  s4 = xTaskCreate(pid_handler_task, "pid_handler_task",
                            configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 3,
                            NULL);
  if(s4 != pdPASS)  Serial.printf("S4 creation problem\r\n");
  s5 = xTaskCreate(pixel_led_task, "pixel_led_task",
                          configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                          NULL);
  if(s5 != pdPASS)  Serial.printf("S5 creation problem\r\n");
  s7 = xTaskCreate(board_support_task, "board_support_task",
                          configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1,
                          NULL);
  if(s7 != pdPASS)  Serial.printf("S7 creation problem\r\n");
  s8 = xTaskCreate(power_board_task, "power_board_task",
                          configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1,
                          NULL);
  if(s8 != pdPASS)  Serial.printf("S8 creation problem\r\n"); 
  
  /* HARDWARE ACTIONS BEFORE RTOS STARTING */
  SetGreenLed(On);
  SetRedLed(Off);
  /* START RTOS */
  if(BOARD_MODE_DEBUG) Serial.printf("Tasks starting\r\n");
  vTaskStartScheduler();
}

static void rclc_spin_task(void *p) {
  UNUSED(p);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, 1);
    switch (uRosLoopHandler())
    {
    case Ok:
      SetGreenLed(On);
      SetRedLed(Off);
      break;
    case Error:
      SetGreenLed(Off);
      SetRedLed(On);
      vTaskDelay(100);
      break;
    default:
      break;
    }
  }
}

static void imu_task(void *p){
  static imu_queue_t queue_imu;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    queue_imu = ImuBno.LoopHandler();
    xQueueSendToFront(ImuQueue, (void*) &queue_imu, TickType_t(0));
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
  double setpoint[] = {0,0,0,0};
  static motor_state_queue_t motor_state;
  static uint8_t freq_div_ptr = 0;
  while(1){
    vTaskDelayUntil(&xLastWakeTime, (1000/PID_FREQ*portTICK_PERIOD_MS));
    xQueueReceive(SetpointQueue, (void*) setpoint, (TickType_t) 0);
    M4_PID.SetSetpoint(setpoint[3]);
    M4_PID.Handler();
    M2_PID.SetSetpoint(setpoint[1]);
    M2_PID.Handler();
    M1_PID.SetSetpoint(setpoint[0]);
    M1_PID.Handler();
    M3_PID.SetSetpoint(setpoint[2]);
    M3_PID.Handler();
    if(freq_div_ptr > (PID_FREQ/MOTORS_RESPONSE_FREQ)){
      motor_state.velocity[0] = (double)M1_PID.Motor->GetVelocity();
      motor_state.velocity[1] = (double)M2_PID.Motor->GetVelocity();
      motor_state.velocity[2] = (double)M3_PID.Motor->GetVelocity();
      motor_state.velocity[3] = (double)M4_PID.Motor->GetVelocity();
      motor_state.positon[0] = (double)M1_PID.Motor->GetPosition();
      motor_state.positon[1] = (double)M2_PID.Motor->GetPosition();
      motor_state.positon[2] = (double)M3_PID.Motor->GetPosition();
      motor_state.positon[3] = (double)M4_PID.Motor->GetPosition();
      xQueueSendToFront(MotorStateQueue, (void*) &motor_state, (TickType_t) 0);
      freq_div_ptr = 0;
    }
    freq_div_ptr++;
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

static void board_support_task(void *p){
  while(1){
    if(digitalRead(PWR_BRD_GPIO_INPUT)){
      //send information to SBC to turn off
    }
    // EXT_SERIAL.println("Hello external device");
    // SBC_SERIAL.println("Hello SBC");
    vTaskDelay(250);
  }
}

static void power_board_task(void *p){
  while(1){
    PowerBoardSerial.UartProtocolLoopHandler();
    vTaskDelay(150);
  }
}

/*============== LOOP - IDDLE TASK ===============*/

void loop() {
  ;
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
