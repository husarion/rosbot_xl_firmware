#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_cfg.h>
/*===== HARDEWARE =====*/
#include <bsp.h>
#include <hardware_cfg.h>
#include <battery_types.h>
// MOTORS
#include <motors.h>
// IMU
#include <ImuLib_cfg.h>
// PIXEL
#if defined(BOARD_ROSBOT_XL)
  #include <PixelLedLib_cfg.h>
#endif
/*===== CONNECTIVITY =====*/
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <UartLib.h>
#include <hal_conf_custom.h>
#include "stm32f407xx.h"

#define PRINT_DEBUG(msg) if (firmware_mode == fw_debug) { Serial.printf(msg); }


/* VARIABLES */
bool uRosInitSuccesfull = false;
// RTOS
QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;
QueueHandle_t BatteryStateQueue;
QueueHandle_t uRosPingAgentStatusQueue;

/* EXTERN VARIABLES */

// IMU
extern ImuDriver ImuBno;
// microROS
extern std_msgs__msg__String msgs;
extern sensor_msgs__msg__Imu imu_msg;
extern sensor_msgs__msg__JointState motors_cmd_msg;
extern sensor_msgs__msg__JointState motors_response_msg;
extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t motor_state_publisher;
// MOTORS
extern TimebaseTimerClass timebase_timer;
extern MotorClass wheel_motors[];

#if defined(BOARD_ROSBOT_XL)
extern UartProtocolClass PowerBoardSerial;
// LED
extern PixelLedClass pixel_strip;
#endif

// ETHERNET
EthernetClient EthClient;

// REST
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;
extern String PowerBoardFirmwareVersion;
extern String PowerBoardVersion;

/* RTOS TASKS DECLARATIONS */
static void RclcSpinTask(void * p);
static void ImuTask(void * p);
static void PidHandlerTask(void * p);
static void PixelLedTask(void * p);
static void SbcShutdownTask(void * p);
static void PowerBoardTask(void * p);
static void uRosPingTask(void * p);
static void HardwareLoopTask(void * p);
static void RuntimeStatsTask(void * p);

/* FUNCTIONS */

/*==================== SETUP ========================*/
void setup()
{
  // Hardware init
  BoardPheripheralsInit();
  uRosTransportInit();
  ImuBno.Init();
  SetGreenLed(On);
  delay(150);
  SetGreenLed(Off);

#if defined(BOARD_ROSBOT_XL)
  PixelStrip.Init();
#endif

  /* RTOS QUEUES CREATION */
  SetpointQueue = xQueueCreate(1, sizeof(double) * 4);
  MotorStateQueue = xQueueCreate(1, sizeof(motor_state_queue_t));
  ImuQueue = xQueueCreate(1, sizeof(imu_queue_t));
  BatteryStateQueue = xQueueCreate(1, sizeof(battery_state_queue_t));
  uRosPingAgentStatusQueue = xQueueCreate(1, sizeof(uRosFunctionStatus));
  PRINT_DEBUG("Queues created\r\n");

  /* RTOS TASKS CREATION */
  if (xTaskCreate(
    RclcSpinTask, "RclcSpinTask", configMINIMAL_STACK_SIZE + 2500, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S1 creation problem\r\n");
  }

  if (xTaskCreate(
    ImuTask, "ImuTask", configMINIMAL_STACK_SIZE + 750, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S2 creation problem\r\n");
  }

  if (xTaskCreate(
    RuntimeStatsTask, "RuntimeStatsTask", configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S3 creation problem\r\n");
  }

  if (xTaskCreate(
    PidHandlerTask, "PidHandlerTask", configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
    PRINT_DEBUG("S4 creation problem\r\n");
  }

#if defined(BOARD_ROSBOT_XL)
  if (xTaskCreate(
    PixelLedTask, "PixelLedTask", configMINIMAL_STACK_SIZE + 750, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S5 creation problem\r\n");
  }

  if (xTaskCreate(
    SbcShutdownTask, "SbcShutdownTask", configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S7 creation problem\r\n");
  }

  if (xTaskCreate(
    PowerBoardTask, "PowerBoardTask", configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S8 creation problem\r\n");
  }
#endif

  if (xTaskCreate(
    uRosPingTask, "uRosPingTask", configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S9 creation problem\r\n");
  }

  if (xTaskCreate(
    HardwareLoopTask, "BoardHardwareLoopTask", configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINT_DEBUG("S10 creation problem\r\n");
    }
  
  /* START RTOS */
  PRINT_DEBUG("Tasks starting\r\n");
  vTaskStartScheduler();
}

static void RclcSpinTask(void * p)
{
  UNUSED(p);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uRosFunctionStatus uRosPingAgentStatus;
  while (1) {
    xQueueReceive(uRosPingAgentStatusQueue, &uRosPingAgentStatus, (TickType_t)0);
    vTaskDelayUntil(&xLastWakeTime, 1);
    uRosLoopHandler(uRosPingAgentStatus);
  }
}

static void ImuTask(void * p)
{
  static imu_queue_t queue_imu;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    queue_imu = ImuBno.LoopHandler();
    xQueueSendToFront(ImuQueue, (void *)&queue_imu, TickType_t(0));
    vTaskDelayUntil(&xLastWakeTime, FREQ_TO_DELAY_TIME(IMU_SAMPLE_FREQ));
  }
}

static void PidHandlerTask(void * p)
{
  TickType_t x_last_wake_time = xTaskGetTickCount();
  TickType_t actual_setpoint_update_time = xTaskGetTickCount();
  TickType_t last_setpoint_update_time = xTaskGetTickCount();
  double setpoint[] = {0, 0, 0, 0};
  static motor_state_queue_t motor_state;
  static uint8_t freq_div_ptr = 0;
  while (1) {
    vTaskDelayUntil(&x_last_wake_time, FREQ_TO_DELAY_TIME(PID_FREQ));
    if (xQueueReceive(SetpointQueue, (void *)setpoint, (TickType_t)0)) {
      last_setpoint_update_time = xTaskGetTickCount();
    }
    actual_setpoint_update_time = xTaskGetTickCount();
    if (actual_setpoint_update_time - last_setpoint_update_time > MOTORS_SETPOINT_TIMEOUT) {
      for (uint8_t i = 0; i < 4; i++) setpoint[i] = 0;
    }
    for (uint8_t i = 0; i < 4; i++) {
      wheel_motors[i].PidLoopHandler((float)setpoint[i]);
    }
    if (freq_div_ptr > (PID_FREQ / MOTORS_RESPONSE_FREQ)) {
      for (uint8_t i = 0; i < 4; i++) {
        motor_state.velocity[i] = ((double)wheel_motors[i].GetVelocity()) / 1000;
        motor_state.position[i] = ((double)(wheel_motors[i].GetWheelAbsPosition()) / 1000);
      }
      xQueueSendToFront(MotorStateQueue, (void *)&motor_state, (TickType_t)0);
      freq_div_ptr = 0;
    }
    freq_div_ptr++;
  }
}

#if defined(BOARD_ROSBOT_XL)
static void PixelLedTask(void * p)
{
  while (1) {
    vTaskDelay(FREQ_TO_DELAY_TIME(PIXEL_ANIMATION_FREQ));
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x0F, 0x0F, 0x0F, 50);
    vTaskDelay(FREQ_TO_DELAY_TIME(PIXEL_ANIMATION_FREQ));
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x00, 0x00, 0x0F, 50);
  }
}

static void SbcShutdownTask(void * p)
{
  IPAddress SbcIpAddr;
  SbcIpAddr.fromString(SBC_AGENT_IP);
  while (1) {
    vTaskDelay(250);
    if (PowerOffSignalLoopHandler() == Shutdown) {
      while (1) {
        if (EthClient.connect(SbcIpAddr, SHUTDOWN_PORT, SBC_ETH_CONNECT_TIMEOUT)) {
          EthClient.println("GET /shutdown HTTP/1.1");
          EthClient.stop();
          vTaskDelay(POWEROFF_DELAY);
          digitalWrite(PWR_BRD_GPIO_OUTPUT, HIGH);
        }
        vTaskDelay(10);
      }
    }
  }
}

static void PowerBoardTask(void * p)
{
  uint16_t TimeDivider = 0;
  while (1) {
    if (PowerBoardFirmwareVersion.length() == 0 || PowerBoardVersion.length() == 0) {
      PbInfoRequest();
    }
    TimeDivider++;
    if (TimeDivider % 5 != 0) BatteryInfoRequest();
    PowerBoardSerial.UartProtocolLoopHandler();
    vTaskDelay(150);
  }
}
#endif

static void uRosPingTask(void * p)
{
  static uRosFunctionStatus uRosPingAgentStatus;
  while (1) {
    uRosPingAgentStatus = uRosPingAgent(PING_AGENT_TIMEOUT, PING_AGENT_ATTEMPTS);
    xQueueSendToFront(uRosPingAgentStatusQueue, (void *)&uRosPingAgentStatus, (TickType_t)0);
    switch (uRosPingAgentStatus) {
      case Ok:
        SetGreenLed(On);
        SetRedLed(Off);
        break;
      case Error:
        SetGreenLed(Off);
        SetRedLed(Toggle);
        break;
      case Default:
        SetGreenLed(Toggle);
        SetRedLed(Off);
        break;
      default:
        SetGreenLed(Off);
        SetRedLed(Off);
        break;
    }
    vTaskDelay(FREQ_TO_DELAY_TIME(PING_AGENT_FREQUENCY));
  }
}

static void HardwareLoopTask(void * p)
{
  vTaskDelay(1000);
#if defined(BOARD_ROSBOT_XL)
  FanHardwareInit();
  while (1) {
    FanLoopHanlder();
    vTaskDelay(100);
  }
#endif
}

static void RuntimeStatsTask(void * p)
{
  char buf[2000];
  PRINT_DEBUG("runtime stats task started\r\n");
  while (1) {
    if (firmware_mode == fw_debug) {
      vTaskGetRunTimeStats(buf);
      Serial.printf("\r\n%s\r\n-------------", buf);
    }
    vTaskDelay(100);
  }
}

/*============== LOOP - IDDLE TASK ===============*/
void loop() { ; }

/*=========== Runtime stats ====================*/
HardwareTimer RuntimeStatsTimer(TIM5);  // TIM5 - 32 bit

void vConfigureTimerForRunTimeStats(void)
{
  RuntimeStatsTimer.setPrescaleFactor(
    1680);  // Set prescaler to 2564 => timer frequency = 168MHz/1680 = 100000
            // Hz (from prediv'd by 1 clocksource of 168 MHz)
  RuntimeStatsTimer.setOverflow(0xffffffff);  // Set overflow to 32761 => timer
                                              // frequency = 65522 Hz / 32761 = 2 Hz
  RuntimeStatsTimer.refresh();                // Make register changes take effect
  RuntimeStatsTimer.resume();                 // Start
}

uint32_t vGetTimerValueForRunTimeStats(void) { return RuntimeStatsTimer.getCount(); }
