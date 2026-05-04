/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

//#error FREERTOS_CPP_IS_BEING_COMPILED

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "encoder.h"
//#include "arm_math.h"
#include "pid.h"
#include "vl6180.h"
#include "speaker.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <cmath>       // For std::abs
#include <stdint.h>    // For uint32_t
#include <stdio.h>     // For printf

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <usart.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STEER_DEG2PWM_RATIO 	0.0009505		// [deg] -> [pwm]
#define STEER_DEG2PWM_OFFSET	0.07620			// [pwm]
#define MOTOR_SPEED_OFFSET 		0.075568        // [pwm]: The zero default where motors are stop
#define M_PPI					6.28318530718	//	2*pi
#define MAX_FORWARD_SPEED		1.0				// [m/s]
#define MAX_BACKWARD_SPEED		-0.5			// [m/s]
#define MAX_FORWARD_PWM_SPEED   0.009
#define MAX_BACKWARD_PWM_SPEED  0.0119
#define MAX_STEERING_ANGLE		28				// [deg]
#define MOT_DECELERATION		-2				// [m/ss]
#define MOT_ACCELERATION		2				// [m/ss]
#define EPSILON					0.01			//

// Filter
//#define FIR_LENGHT 		 		13

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
double speedTarget=0.0, speedTargetPos=0.0, speedRef=0.0; 									// [m/s], [m/s]: speed references
double steerRef=0.0;																		// [deg]: steer references
// MEASUREMENT variables
double carSpeed=0.0;																		// [rps], [m/s], [rps]: speed measurements
double carSpeedfiltered=0.0;// [m/s]
double encoderAcceleration = 0.0;
double globalDistance=0.0, localDistance=0.0, localDistanceOrigin=0.0, localDistanceRef=0.0;// [m]: position measurements
long encoderCount=0;																		// [tick]: encoder count
long encoderTotal=0;
// CONTROL variables
double motorPWM=0.0, servoPWM=0.0;// [tick], [tick]: control variables
// STATE of the CAR
bool isSpeedControl = true;
bool isPositionControl = false;
volatile bool posAckDone=false, reachedPosition=false, pub_flag=false;
volatile bool checkEmergencyBrakeArena = false;
bool inRangeForEmergencyBrakeArena = false;
bool flagLed = false;


////Finite Impulse Response filter
//float32_t fir_coefficients[FIR_LENGHT] = {
//		0.0120f, 0.0213f, 0.0470f, 0.0822f,
//		0.1175f, 0.1435f, 0.1531f, 0.1435f,
//		0.1175f, 0.0822f, 0.0470f, 0.0213f,
//		0.0120f
//};
//arm_fir_instance_f32 fir_instance;
//float32_t fir_in_arm, fir_out_arm, fir_state[FIR_LENGHT];

// VL6180X TOF sensor definition
VL6180X_Sensor sensorsTOF[NUMBER_OF_VL6180X_ID];
uint8_t TOFcounterToInitialize = 0;
uint8_t TenMsCounter = 0;


uint8_t samplingPID = (uint8_t)((ENC_COUNT_MAX + 1) / 1e3); // [ms]

// CONTROLLERS PID 			 (Input,     Output,    Setpoint) 0.020001, 0.0250001, 0.000
PID SpeedController 	= PID(&carSpeedfiltered, &motorPWM, &speedRef, 0.020001, 0.0250001, 0,_PID_P_ON_E, _PID_CD_DIRECT);// PID_Proportional_On_Error
PID PositionController 	= PID(&localDistance, &speedTargetPos, &localDistanceRef, 6, 2, 0,_PID_P_ON_E, _PID_CD_DIRECT);// PID_Proportional_On_Error
// 6 2 4 or 6 2 3 for pos


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern ADC_HandleTypeDef hadc1;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[1000];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void speed_callback(const void * msgin);
void steer_callback(const void * msgin);
void stop_callback(const void * msgin);
void pos_callback(const void * msgin);
void checkEmergencyBrakeArena__callback(const void * msgin);
void leds_callback(const void * msgin);

void stop(float angle);
void steer(float angle);
void speed(float vel);
void position(float dist);
void drive_pwm(float pwm_value);

bool isClose(double A, double B);
double last_diff=0.0;

float readADC(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */


/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  if (defaultTaskHandle == NULL)
  {
      Error_Handler();
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  /* USER CODE BEGIN 5 */
  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);


  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      //printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rclc_executor_t executor;

  rcl_publisher_t carSpeed_pub;
  rcl_publisher_t globalDistance_pub;
  rcl_publisher_t acceleration_pub;
  rcl_publisher_t posFeedback_pub;
  rcl_publisher_t tof_front_pub;
  rcl_publisher_t tof_left_pub;

  // micro-ROS Subscriber handles
  rcl_subscription_t speed_sub;
  rcl_subscription_t steer_sub;
  rcl_subscription_t pos_sub;
  rcl_subscription_t stop_sub;

//  rcl_subscription_t checkEmergencyBrakeArena_sub;
  rcl_subscription_t leds_sub;

  std_msgs__msg__Float32 speed_msg_in;
  std_msgs__msg__Float32 steer_msg_in;
  std_msgs__msg__Float32 pos_msg_in;
  std_msgs__msg__Float32 stop_msg_in;

  //std_msgs__msg__Bool checkEmergencyBrakeArena_msg_in;
  std_msgs__msg__Bool leds_msg_in;

  std_msgs__msg__Float32 carSpeed_msg_out;
  std_msgs__msg__Float32 dist_msg_out;
  std_msgs__msg__Float32 accel_msg_out;
  std_msgs__msg__Bool posFeedback_msg_out;
  std_msgs__msg__UInt8 tof_front_msg_out;
  std_msgs__msg__UInt8 tof_left_msg_out;

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create automobile node
  rclc_node_init_default(&node, "automobile_node", "", &support);

  // create publishers
  rclc_publisher_init_default(&carSpeed_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "automobile/encoder/speed");
  rclc_publisher_init_default(&globalDistance_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "automobile/encoder/distance");
  rclc_publisher_init_default(&posFeedback_pub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "automobile/feedback/position");
  rclc_publisher_init_default(&tof_front_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "automobile/tof/front");
  rclc_publisher_init_default(&tof_left_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "automobile/tof/left");
  rclc_publisher_init_default(&acceleration_pub,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"automobile/encoder/acceleration");

  // create subscribers
  rclc_subscription_init_default(&speed_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "automobile/command/speed");
  rclc_subscription_init_default(&steer_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "automobile/command/steer");
  rclc_subscription_init_default(&pos_sub, &node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "automobile/command/position");
  rclc_subscription_init_default(&stop_sub, &node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "automobile/command/stop");
  //rclc_subscription_init_default(&checkEmergencyBrakeArena_sub, &node,
    //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "automobile/arena");
  rclc_subscription_init_default(&leds_sub, &node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "automobile/led");

  rclc_executor_init(&executor, &support.context, 5, &allocator);

  rclc_executor_add_subscription(&executor, &speed_sub, &speed_msg_in, &speed_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &steer_sub, &steer_msg_in, &steer_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pos_sub, &pos_msg_in, &pos_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &stop_sub, &stop_msg_in, &stop_callback, ON_NEW_DATA);
  //rclc_executor_add_subscription(&executor, &checkEmergencyBrakeArena_sub, &checkEmergencyBrakeArena_msg_in, &checkEmergencyBrakeArena__callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &leds_sub, &leds_msg_in, &leds_callback, ON_NEW_DATA);

  // Initialize the actual message structures, not the handles
  std_msgs__msg__Float32__init(&speed_msg_in);
  std_msgs__msg__Float32__init(&steer_msg_in);
  std_msgs__msg__Float32__init(&pos_msg_in);
  std_msgs__msg__Float32__init(&stop_msg_in);
  //std_msgs__msg__Bool__init(&checkEmergencyBrakeArena_msg_in);
  std_msgs__msg__Bool__init(&leds_msg_in);

  SpeedController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
  SpeedController.SetOutputLimits(-0.009, 0.0119);// PWM Output limits
  SpeedController.SetSampleTime(samplingPID);// set sample time in [ms]
  SpeedController.EnableProportionalFilter(true);
  SpeedController.UseIFilter(true);
  //SpeedController.SetProportionalFilterWindow(0);

  // Set PID POSITION controller parameters
  PositionController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
  PositionController.SetOutputLimits(-0.2, 0.2);// Speed Output limits [m/s]
  PositionController.SetSampleTime(samplingPID);// set sample time in [ms]
  PositionController.EnableProportionalFilter(false);

  //initialize the filter
//  arm_fir_init_f32(&fir_instance, FIR_LENGHT, fir_coefficients, fir_state, 1);

  // VL6180X TOF sensors initialization
  VL6180X_Setup(&sensorsTOF[0]);
  VL6180X_i2c_Init(&sensorsTOF[0]);
  VL6180X_ReadAllDistancesPolling(&sensorsTOF[0], &TOFcounterToInitialize);
  //VL6180X_Setup(&sensorsTOF[0]);
  //VL6180X_i2c_Init(&sensorsTOF[0]);
  //VL6180X_ReadAllDistances(&sensorsTOF[0], &TOFcounterToInitialize);

 if (TOFcounterToInitialize != 0){
  	//printf("Error initializig TOF sensors! \r\n");
  }

  // 1. Overwrite the bad CubeMX settings
  TIM2->ARR = 39999;
  TIM2->PSC = 41;
  TIM2->EGR = TIM_EGR_UG; // Update registers

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // DC MOTOR PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // SERVO PWM
  HAL_TIM_Base_Start(&htim4); // LED and SPEAKERS
  Encoder_Init(&htim3);
  HAL_TIM_Base_Start_IT(&htim11); // PID SPEED CONTROL
  HAL_TIM_Base_Start_IT(&htim10); // PUBLISH TIMER


  speedTarget = 0.0;
  speedRef 	= 0.0;
  steerRef 	= 0.0;
  //added
  steer(0);
  drive_pwm(0);


  for(;;)
  {
	  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

	      if (ret == RCL_RET_OK) {

	          if (pub_flag) {
	        	  carSpeed_msg_out.data = (float)carSpeedfiltered;
				  rcl_publish(&carSpeed_pub, &carSpeed_msg_out, NULL);

				  // Distance
				  dist_msg_out.data = (float)globalDistance;
				  rcl_publish(&globalDistance_pub, &dist_msg_out, NULL);

				  accel_msg_out.data = (float)encoderAcceleration;;
				  rcl_publish(&acceleration_pub, &accel_msg_out, NULL);

				  if (reachedPosition && !posAckDone) {
					  posFeedback_msg_out.data = true;
					  rcl_publish(&posFeedback_pub, &posFeedback_msg_out, NULL);
					  posAckDone = true;
				  }


				  tof_front_msg_out.data = sensorsTOF[1].distance;
				  rcl_publish(&tof_front_pub, &tof_front_msg_out, NULL);

				  tof_left_msg_out.data = sensorsTOF[0].distance;
				  rcl_publish(&tof_left_pub, &tof_left_msg_out, NULL);

				  // Reset the flag (set by your 10ms Timer Interrupt)
				  pub_flag = false;

	          }
	      }

	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}


//void StartDefaultTask(void *argument)
//{
//  for (;;)
//  {
//    HAL_UART_Transmit(&huart2, (uint8_t*)"FREERTOS HELLO\r\n", 16, 100);
//    osDelay(1000);
//  }
//}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



void speed_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    speed(msg->data); // Your existing speed() function
}

void steer_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    steer(msg->data); // Your existing steer() function
}

void pos_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    position(msg->data); // Your existing position() function
}

void stop_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    stop(msg->data); // stop existing speed() function
}

void checkEmergencyBrakeArena__callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    checkEmergencyBrakeArena = msg->data;
}

void leds_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    flagLed = msg->data;
    	// turn on/off leds
    	if (flagLed) {
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    	} else {
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	}
    }




void stop(float angle){
	// ---------- STATE ----------
	isPositionControl 	= false;
	isSpeedControl 		= false;

	// ---------- STOP references ----------
	speedTarget = 0.0;
	speedTargetPos = 0.0;
	speedRef = 0.0;

	// ---------- STOP ----------
	motorPWM = 0.0;
	steer(angle);
}

void speed(float vel){
	// ---------- SPEED references ----------
	if(vel > MAX_FORWARD_SPEED){
		speedTarget = MAX_FORWARD_SPEED;
	}
	else if (vel < MAX_BACKWARD_SPEED){
		speedTarget = MAX_BACKWARD_SPEED;
	}
	else{
		speedTarget = vel;
	}

	// ---------- STATE ----------
	isSpeedControl 		= true;
	isPositionControl 	= false;

}

void steer(float angle){

	// ---------- STEER references ----------
	if(angle > MAX_STEERING_ANGLE){steerRef = MAX_STEERING_ANGLE;}
	else if (angle < -MAX_STEERING_ANGLE){steerRef = -MAX_STEERING_ANGLE;}
	else{steerRef = angle;}

	// ---------- STEER ----------
	servoPWM = STEER_DEG2PWM_RATIO * steerRef + STEER_DEG2PWM_OFFSET;			// [deg] -> [pwm]
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servoPWM*SERVO_PWM_COUNT_MAX);	// set servo pwm
}

void position(float dist){
	// ---------- POSITION references ----------
	localDistanceOrigin = globalDistance;
	localDistanceRef = dist;

	// ---------- STATE ----------
	isSpeedControl = true;
	isPositionControl = true;
	posAckDone = false;
	reachedPosition = false;
}

void drive_pwm(float pwm_value){
	// ---------- ACTUATION - SPEED ----------

	// is on reverse
	pwm_value = -pwm_value;

	// Range pwm_value - values are calibrated to reach a max of 1.0 m/s forward and 0.5 backwards
	if (pwm_value > 0.009f) pwm_value = 0.009f; 	//Backwards
	if (pwm_value < -0.0119f) pwm_value = -0.0119f; //Forward

	// considering the default zero speed is at 7.5568% duty cycle
	float duty_cycle = (pwm_value + MOTOR_SPEED_OFFSET);

	// Convert duty cycle to pulse width and set the PWM value
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(duty_cycle * MOT_PWM_COUNT_MAX));
}

bool isClose(double A, double B)
{
	double diff = A - B;
	return (abs(diff) < 0.001) ;//&& (abs(last_diff-diff) < EPSILON_DIFF) ;
	//last_diff = diff;
}

float readADC(void)
{
	uint32_t  adc_value;
	float scaled_value;

	HAL_ADC_Start(&hadc1); // Start ADC conversion
	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	{ // Wait for conversion
		adc_value = HAL_ADC_GetValue(&hadc1); // Read ADC value
		scaled_value = (float)adc_value;// / 7358.54; // Scale ADC value
	}
	HAL_ADC_Stop(&hadc1); // Stop ADC conversion

	return scaled_value;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// ---------- USER BUTTON EMERGENCY BRAKE (BLUE BUTTON)----------
	if(GPIO_Pin == B1_Pin){

		stop(0.0);
        // Used for scalling the Graph in debugging, press twice the blue button so it gets scalled
		/*
		if (counter % 2 == 0){
			input=-1;
			output=-1;
			setpoint=-1;
		}
		else{
			input=1;
			output=1;
			setpoint=1;
		}
		counter++;
		 */


		//Update PID gains
		//printf("KP = %f , KI = %f , KD = %f \r\n",KP,KI,KD);
		//SpeedController.EnableProportionalFilter(false);
		//SpeedController.SetTunings((double)KP,(double)KI,(double)KD);
		//SpeedController.SetProportionalFilterWindow(pFilterWindow);
		//SpeedController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
		//SpeedController.SetOutputLimits(-0.009, 0.0119);// PWM Output limits
		//SpeedController.SetSampleTime(samplingPID);// set sample time in [ms]

		//PositionController.SetTunings((double)KP,(double)KI,(double)KD);
		//PositionController.SetMode(_PID_MODE_AUTOMATIC);// AUTOMATIC MODE
		//PositionController.SetOutputLimits(-0.2, 0.2);// PWM Output limits
		//PositionController.SetSampleTime(samplingPID);// set sample time in [ms]

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim == &htim11){  //every 1ms, PSC = 83, ARR = 999, f_tim = 84MHz -> (ARR+1*PSC)/f_TIM = 0.001s = 1ms

		Encoder_Update_ISR();

		Encoder_State_t enc = Encoder_GetState();

		encoderCount = enc.delta_counts;
		encoderTotal = enc.total_counts_corrected;

		carSpeed = enc.velocity_mps;
		carSpeedfiltered = enc.velocity_mps_filtered;
		globalDistance = enc.distance_m_filtered;
		encoderAcceleration = enc.acceleration_mps2;
		localDistance = globalDistance - localDistanceOrigin;


		if (sensorsTOF[1].distance > 5 && sensorsTOF[1].distance < 80){
			inRangeForEmergencyBrakeArena = true;
		}
		if (inRangeForEmergencyBrakeArena == true && sensorsTOF[1].distance > 100){
			inRangeForEmergencyBrakeArena = false;
		}

		if (!checkEmergencyBrakeArena || !inRangeForEmergencyBrakeArena){

			// ---------- DOUBLE LOOP CONTROL ----------
			// ---------- OUTER LOOP: POSITION CONTROL - PID ----------
			if(isPositionControl){

				if(isClose(localDistance, localDistanceRef) &&  !posAckDone){
					reachedPosition = true;
					speedTarget = 0.0;
					isPositionControl = false; //just added
				}
				else{
					reachedPosition = false;
					PositionController.Compute();
					speedTarget = speedTargetPos;
				}
			}

			// ---------- INNER LOOP: SPEED CONTROL - PID ----------
			if(isSpeedControl){

				speedRef = speedTarget;
				SpeedController.Compute();// SPEED control action
			}


			// ---------- ACTUATION - SPEED ----------

			if(abs(speedRef) == 0.00 || abs(motorPWM)<0.00001)
				drive_pwm(0);
			else
				speedRef = speedRef;
//				printf("s\n\r");//drive_pwm(motorPWM);
		} else {
			//stop(steerRef);
			drive_pwm(0);
		}

		drive_pwm(speedTarget);


		// Drive with pwm from topics
		//drive_pwm(speedTarget);

		//for plotting on SWM graph
		//input=(float32_t)carSpeed;
		//output=(float)motorPWM * 100;
		//setpoint=(float)speedRef;

//		input=(float)fir_out_arm*100;
//		output=(float)motorSpeed * 100;

		//input=(float)localDistance;
		//output=(float)speedTargetPos;
		//setpoint=(float)localDistanceRef;

		// ---------- RESET ENCODER COUNTER ----------
		//__HAL_TIM_SET_COUNTER(&htim3,0);

	}

	if(htim == &htim10){  //every 10ms
		pub_flag = true;

		/* For TOF sensors */

		TenMsCounter++;


		if (TenMsCounter == 1){
			/* at millisecond 10 prepare sensor to read values */
			VL6180X_SendMsgToRead(&sensorsTOF[0]);
		} else if (TenMsCounter == 8){
			/* at millisecond 70 read values */
			VL6180X_ReadDistances(&sensorsTOF[0], &TOFcounterToInitialize);
			//VL6180X_PrintAllDistances(&sensorsTOF[0]);
			TenMsCounter = 0; // Reset counter
		}
	}

	//if (TenMsCounter == 1){
		/* at millisecond 10 prepare sensor to read values */
	//	VL6180X_SendMsgToRead(&sensorsTOF[0]);
	//}
	//if (TenMsCounter == 8){
		/* at millisecond 70 read values */
	//	VL6180X_ReadDistances(&sensorsTOF[0], &TOFcounterToInitialize);
		//VL6180X_PrintAllDistances(&sensorsTOF[0]);
	//	TenMsCounter = 0; // Reset counter
	//}

  /* USER CODE END Callback 1 */
}
#ifdef __cplusplus
}
#endif
/* USER CODE END Application */

