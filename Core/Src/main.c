/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDR		0xD0

#define SMPLRT_DIV			0x19

#define MPU9250_CONFIG		0X1A
#define GYRO_CONFIG			0X1B
#define ACCEL_CONFIG		0X1C
#define ACCEL_CONFIG_2		0X1D

#define ACCEL_XOUT_H		0X3B
#define ACCEL_XOUT_L		0X3C
#define ACCEL_YOUT_H		0X3D
#define ACCEL_YOUT_L		0X3E
#define ACCEL_ZOUT_H		0X3F
#define ACCEL_ZOUT_L		0X40

#define TEMP_OUT_H			0X41
#define TEMP_OUT_L			0x42

#define GYRO_XOUT_H			0X43
#define GYRO_XOUT_L			0X44
#define GYRO_YOUT_H			0X45
#define GYRO_YOUT_L			0X46
#define GYRO_ZOUT_H			0X47
#define GYRO_ZOUT_L			0X48

#define PWR_MGMT_1			0x6B
#define WHO_AM_I_REG 		0x75





/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t GYRO_DLPF_CFG		= 0X02; 	// Set digital low-pass filter 0x02 to get bandwidth 92Hz
uint8_t ACCEL_DLPF_CFG		= 0X02;		// Set digital low-pass filter 0x02 to get bandwidth 99Hz

uint8_t GYRO_FS				= 0x08;		// +500DPS
uint8_t ACCEL_FS			= 0X10;		// +-8G

uint8_t PWR_CONIG			= 0X00;		// Auto select the best available clock source
uint8_t SMPLRT_CONFIG		= 0X07;		// Sample_Rate = Internal_Sample_Rate/(1+SMPLRT_CONFIG)
										// Sample_Rate = 8Khz/(1+8) = 1Khz
uint8_t MPU9250_RESPONSE_OK	= 0x71;		// Check WHO_AM_I if it is OK it will 0x71 or 113
uint8_t MPU9250_Check		= 0;

bool set_gyro_angle = false;

volatile long ch[8];
volatile long tick;
volatile uint8_t pulse;

int receiver_input_channel_1;
int receiver_input_channel_2;
int receiver_input_channel_3;
int receiver_input_channel_4;
int receiver_input_channel_5;
int receiver_input_channel_6;

int throttle;

long gyro_x_cal;
long gyro_y_cal;
long gyro_z_cal;

int loop_timer;

int gyro_x;
int gyro_y;
int gyro_z;


long acc_x;
long acc_y;
long acc_z;
long acc_total_vector;

float angle_roll;
float angle_pitch;

float angle_roll_output;
float angle_pitch_output;

float angle_roll_acc;
float angle_pitch_acc;

float roll_level_adjust;
float pitch_level_adjust;

float gyro_roll_input;
float gyro_pitch_input;
float gyro_yaw_input;

float pid_roll_setpoint;
float pid_pitch_setpoint;
float pid_yaw_setpoint;


float pid_p_gain_roll		= 1.68;			//1.5
float pid_i_gain_roll		= 0.055;			//0.02
float pid_d_gain_roll		= 15;			//15

float pid_p_gain_pitch		= 1.68;			//1.5
float pid_i_gain_pitch		= 0.055;			//0.02
float pid_d_gain_pitch		= 15;			//15

float pid_p_gain_yaw		= 0.03;			//0.02
float pid_i_gain_yaw		= 0.0002;
float pid_d_gain_yaw		= 10;			//15

float pid_i_mem_roll;
float pid_i_mem_pitch;
float pid_i_mem_yaw;

float pid_last_roll_d_error;
float pid_last_pitch_d_eroor;
float pid_last_yaw_d_error;

float pid_error_temp;

float pid_roll_output;
float pid_pitch_output;
float pid_yaw_output;

int pid_max_roll			= 400;
int pid_max_pitch			= 400;
int pid_max_yaw				= 400;

int esc_1;
int esc_2;
int esc_3;
int esc_4;

int start;

int min_throthle = 1070;
int max_throthle = 2000;
int disable_motor = 1000;


float turning_speed = 5.0;
bool auto_level = true;

uint8_t incomingMsg[100];
uint8_t message_counter;
uint8_t rx_buf;

uint8_t num_used_sats;
uint8_t latitude_north;
uint8_t longtitde_east;
long lat_gps_actual;
long lon_gps_actual;


bool new_line_found = false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void gps_setup(){

	uint8_t disableGPGSV[11] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
	uint8_t setTo_5Hz[14] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
	uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
	                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
	                              };
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1, disableGPGSV, 11,1000);
	HAL_Delay(300);
	HAL_UART_Transmit(&huart1, setTo_5Hz, 14, 1000);
	HAL_Delay(300);
	HAL_UART_Transmit(&huart1, Set_to_57kbps, 28, 10000);
	HAL_Delay(300);

}
void gps_read(){

	HAL_UART_Receive(&huart1, &rx_buf,1,1000);

	if ( rx_buf == '$'){
		for (message_counter =0 ; message_counter <=99; message_counter ++){
			incomingMsg[message_counter] = '-';
		}

		message_counter =0;
	}

	else if ( message_counter <= 99 ) message_counter++;
	incomingMsg[message_counter] = rx_buf;
	if ( rx_buf == '*' ) new_line_found = true;

	if ( new_line_found ){
		new_line_found = false;

		if ( incomingMsg[4] == 'L' && incomingMsg[5] == 'L' && incomingMsg[7] == ',' ){
			HAL_GPIO_TogglePin(led_status_GPIO_Port,led_status_Pin);



		}

		if ( incomingMsg[4] == 'G' && incomingMsg[5] == 'A' && (incomingMsg[44] == '1' || incomingMsg[44] == '2')){

			lat_gps_actual = ((int) incomingMsg[19] - 48) *  (long) 10000000;
			lat_gps_actual += ((int) incomingMsg[20] - 48) * (long) 1000000;
			lat_gps_actual += ((int) incomingMsg[22] - 48) * (long) 100000;
			lat_gps_actual += ((int) incomingMsg[23] - 48) * (long) 10000;
			lat_gps_actual += ((int) incomingMsg[24] - 48) * (long) 1000;
			lat_gps_actual += ((int) incomingMsg[25] - 48) * (long) 100;
			lat_gps_actual += ((int) incomingMsg[26] - 48) * (long) 10;

			lat_gps_actual /= (long) 6;

			lat_gps_actual += ((int) incomingMsg[17] - 48) * (long)100000000;
			lat_gps_actual+= ((int) incomingMsg[18] - 48) *  (long)10000000;

			lat_gps_actual /= 10;

			lon_gps_actual = ((int) incomingMsg[33] - 48) *  (long) 10000000;
			lon_gps_actual += ((int) incomingMsg[34] - 48) * (long) 1000000;
			lon_gps_actual += ((int) incomingMsg[36] - 48) * (long) 100000;
			lon_gps_actual += ((int) incomingMsg[37] - 48) * (long) 10000;
			lon_gps_actual += ((int) incomingMsg[38] - 48) * (long) 1000;
			lon_gps_actual += ((int) incomingMsg[39] - 48) * (long) 100;
			lon_gps_actual += ((int) incomingMsg[40] - 48) * (long) 10;

			lon_gps_actual /= (long) 6;

			lon_gps_actual += ((int) incomingMsg[30] - 48) * (long)100000000;
			lon_gps_actual+= ((int) incomingMsg[31] - 48) *  (long)10000000;
			lon_gps_actual+= ((int) incomingMsg[32] - 48) *  (long)1000000;
			lon_gps_actual /= 10;

		}
	}

}
void gyro_setup(){

	 if (HAL_I2C_Mem_Read (&hi2c1, MPU9250_ADDR,WHO_AM_I_REG,1, &MPU9250_Check, 1, 1000) == HAL_OK ){
		 if ( MPU9250_Check == MPU9250_RESPONSE_OK ){

			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1, 1, &PWR_CONIG, 1, 100);				// Auto select best available clock source
			 HAL_Delay(100);
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, SMPLRT_DIV, 1, &SMPLRT_CONFIG, 1, 100);			// 1Khz sample rate

			 HAL_Delay(100);
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, MPU9250_CONFIG, 1, &GYRO_DLPF_CFG, 1, 100);		// Set digital low-pass filter 0x02 to get bandwidth 92Hz
			 HAL_Delay(100);
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG_2, 1, &ACCEL_DLPF_CFG, 1, 100);		// Set digital low-pass filter 0x02 to get bandwidth 99Hz

			 HAL_Delay(100);
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG, 1, &GYRO_FS, 1, 100);					// Set GYRO to full scale +250DPS
			 HAL_Delay(100);
			 HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG, 1, &ACCEL_FS, 1, 100);				// Set ACCEL to full scale +-2G


		 }

	 }

}

void gyro_get_data(){

	uint8_t Accel_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H, 1, Accel_Val_Raw, 6, 1000);

	acc_y = (int16_t) (Accel_Val_Raw[0] << 8 | Accel_Val_Raw [1]);
	acc_x = (int16_t) (Accel_Val_Raw[2] << 8 | Accel_Val_Raw [3]);
	acc_z = (int16_t) (Accel_Val_Raw[4] << 8 | Accel_Val_Raw [5]);

	uint8_t Gyro_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, GYRO_XOUT_H, 1, Gyro_Val_Raw, 6, 1000);

	gyro_y = (int16_t) (Gyro_Val_Raw[0] << 8 | Gyro_Val_Raw [1]);
	gyro_x = (int16_t) (Gyro_Val_Raw[2] << 8 | Gyro_Val_Raw [3]);
	gyro_z = (int16_t) (Gyro_Val_Raw[4] << 8 | Gyro_Val_Raw [5]);

	gyro_x *= -1;
//	gyro_y *= -1;
	gyro_z *= -1;

}

void gyro_cal(){

	for( int i = 0; i < 2000; i++){
		if ( i % 15 == 0 ) HAL_GPIO_TogglePin(led_cal_GPIO_Port, led_cal_Pin);
		gyro_get_data();

		gyro_x_cal += gyro_x;
		gyro_y_cal += gyro_y;
		gyro_z_cal += gyro_z;

		HAL_Delay(4);

	}

	gyro_x_cal /= 2000;
	gyro_y_cal /= 2000;
	gyro_z_cal /= 2000;

}





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  HAL_TIM_Base_Start(&htim13);
  HAL_TIM_Base_Start(&htim14);


  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_Delay(100);

  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,0);

  HAL_Delay(100);
  gyro_setup();
  HAL_Delay(100);
  gyro_cal();
  HAL_Delay(50);
//  gps_setup();

  loop_timer = __HAL_TIM_GET_COUNTER(&htim13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  gps_read();

	  receiver_input_channel_1 = ch[0];
	  receiver_input_channel_2 = ch[1];
	  receiver_input_channel_3 = ch[2];
	  receiver_input_channel_4 = ch[3];
	  receiver_input_channel_5 = ch[4];
	  receiver_input_channel_6 = ch[5];

	  gyro_get_data();

	  gyro_x -= gyro_x_cal;
	  gyro_y -= gyro_y_cal;
	  gyro_z -= gyro_z_cal;


	  gyro_pitch_input 	= ( gyro_pitch_input * 0.7 ) + ((float)( gyro_x / 65.5) * 0.3);
	  gyro_roll_input 	= ( gyro_roll_input * 0.7 ) + ((float)( gyro_y / 65.5) * 0.3);
	  gyro_yaw_input 	= ( gyro_yaw_input * 0.7 ) + ((float)( gyro_z / 65.5) * 0.3);

	  angle_pitch += gyro_x * 0.0000611;
	  angle_roll += gyro_y * 0.0000611;


	  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);
	  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);

	  acc_total_vector = sqrt( ( acc_x*acc_x ) + ( acc_y * acc_y) + ( acc_z * acc_z ) );

	  if ( abs(acc_y) < acc_total_vector ){
		  angle_pitch_acc = asin( (float) acc_y / acc_total_vector ) * 57.296;
	  }

	  if ( abs(acc_x) < acc_total_vector ){
		  angle_roll_acc = asin( (float) acc_x / acc_total_vector ) * 57.296;
	  }


	  angle_pitch_acc -= 0.0;		// -1
	  angle_roll_acc -= 0.0;		// -2.5



	  if ( set_gyro_angle ) {
		  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
		  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
	  }
	  else{
		  angle_pitch = angle_pitch_acc;
		  angle_roll = angle_roll_acc;
		  set_gyro_angle = true;
	  }

//	  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
//	  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;


//	  pitch_level_adjust = angle_pitch_output * 15;
//	  roll_level_adjust = angle_roll_output * 15;

	  if ( !auto_level ){
		  pitch_level_adjust =0;
		  roll_level_adjust =0;
	  }


	  if ( receiver_input_channel_3 < 1050 && receiver_input_channel_1 < 1050 ) start =1;

	  if ( start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_1 > 1450 ){
		  start = 2;
		  HAL_GPIO_WritePin(led_arm_GPIO_Port, led_arm_Pin, 1);
		  HAL_GPIO_WritePin(led_disarm_GPIO_Port, led_disarm_Pin, 0);

		  pid_i_mem_roll = 0;
		  pid_last_roll_d_error = 0;
		  pid_i_mem_pitch = 0;
		  pid_last_pitch_d_eroor = 0;
		  pid_i_mem_yaw = 0;
		  pid_last_yaw_d_error = 0;
	  }

	  if ( start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_1 > 1950 ){
		  start =0;
		  HAL_GPIO_WritePin(led_arm_GPIO_Port, led_arm_Pin, 0);
		  HAL_GPIO_WritePin(led_disarm_GPIO_Port, led_disarm_Pin, 1);
	  }

	  if ( receiver_input_channel_5 > 1500 ) turning_speed = 5;
	  else turning_speed = 3;


	  pid_roll_setpoint =0;
	  if ( receiver_input_channel_4 > 1508 ) pid_roll_setpoint = (receiver_input_channel_4 - 1508);
	  else if ( receiver_input_channel_4  < 1492 ) pid_roll_setpoint = ( receiver_input_channel_4  - 1492 );

	  pid_roll_setpoint -= roll_level_adjust;
	  pid_roll_setpoint /= turning_speed;

	  pid_pitch_setpoint =0;
	  if ( receiver_input_channel_2 > 1508 ) pid_pitch_setpoint = ( receiver_input_channel_2 - 1508 );
	  else if ( receiver_input_channel_2 < 1492 ) pid_pitch_setpoint = ( receiver_input_channel_2 - 1492 );

	  pid_pitch_setpoint -= pitch_level_adjust;
	  pid_pitch_setpoint /= turning_speed;


	  pid_yaw_setpoint =0;
	  if ( receiver_input_channel_3 > 1050 ){
		  if ( receiver_input_channel_1 > 1508 ) pid_yaw_setpoint = ( receiver_input_channel_1 - 1508 ) / turning_speed;
		  else if ( receiver_input_channel_1 < 1492 ) pid_yaw_setpoint = ( receiver_input_channel_1 - 1492 ) / turning_speed;
	  }

	  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

	  if ( pid_i_mem_roll > pid_max_roll ) pid_i_mem_roll = pid_max_roll;
	  else if ( pid_i_mem_roll < pid_max_roll * -1 ) pid_i_mem_roll = pid_max_roll * -1;

	  pid_roll_output = ( pid_p_gain_roll * pid_error_temp ) + pid_i_mem_roll + ( pid_d_gain_roll * ( pid_error_temp - pid_last_roll_d_error));

	  if ( pid_roll_output > pid_max_roll ) pid_roll_output = pid_max_roll;
	  else if ( pid_roll_output < pid_max_roll * -1) pid_roll_output = pid_max_roll * -1;

	  pid_last_roll_d_error = pid_error_temp;


	  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

	  if ( pid_i_mem_pitch > pid_max_pitch ) pid_i_mem_pitch = pid_max_pitch;
	  else if ( pid_i_mem_pitch < pid_max_pitch * -1 ) pid_i_mem_pitch = pid_max_pitch * -1;

	  pid_pitch_output = ( pid_p_gain_pitch * pid_error_temp ) + pid_i_mem_pitch + ( pid_d_gain_pitch * ( pid_error_temp - pid_last_pitch_d_eroor));

	  if ( pid_pitch_output > pid_max_pitch ) pid_pitch_output = pid_max_pitch;
	  else if ( pid_pitch_output < pid_max_pitch * -1 ) pid_pitch_output = pid_max_pitch * -1;

	  pid_last_pitch_d_eroor = pid_error_temp;


	  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	  pid_i_mem_yaw += pid_p_gain_yaw * pid_error_temp;

	  if ( pid_i_mem_yaw > pid_max_yaw ) pid_i_mem_yaw = pid_max_yaw;
	  else if ( pid_i_mem_yaw < pid_max_yaw * -1 ) pid_i_mem_yaw = pid_max_yaw * -1;

	  pid_yaw_output = ( pid_p_gain_yaw * pid_error_temp ) + pid_i_mem_yaw + ( pid_d_gain_yaw * ( pid_error_temp - pid_last_yaw_d_error ));

	  if ( pid_yaw_output > pid_max_yaw ) pid_yaw_output = pid_max_yaw;
	  else if ( pid_yaw_output < pid_max_yaw * -1 ) pid_yaw_output = pid_max_yaw * -1;

	  pid_last_yaw_d_error = pid_error_temp;

	  throttle = receiver_input_channel_3;


	  if ( start == 2 ){
		  if ( throttle > 1900 ) throttle = 1900;

		  esc_1 = throttle - pid_pitch_output + pid_roll_output - pid_yaw_output;
		  esc_2 = throttle + pid_pitch_output + pid_roll_output + pid_yaw_output;
		  esc_3 = throttle + pid_pitch_output - pid_roll_output - pid_yaw_output;
		  esc_4 = throttle - pid_pitch_output - pid_roll_output + pid_yaw_output;

		  if ( esc_1 < min_throthle ) esc_1 = min_throthle;
		  if ( esc_2 < min_throthle ) esc_2 = min_throthle;
		  if ( esc_3 < min_throthle ) esc_3 = min_throthle;
		  if ( esc_4 < min_throthle ) esc_4 = min_throthle;

		  if ( esc_1 > max_throthle ) esc_1 = max_throthle;
		  if ( esc_2 > max_throthle ) esc_2 = max_throthle;
		  if ( esc_3 > max_throthle ) esc_3 = max_throthle;
		  if ( esc_4 > max_throthle ) esc_4 = max_throthle;


	  }else{
		  esc_1 = disable_motor;
		  esc_2 = disable_motor;
		  esc_3 = disable_motor;
		  esc_4 = disable_motor;
	  }

	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,esc_1);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,esc_2);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,esc_3);
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,esc_4);

//	  if ( __HAL_TIM_GET_COUNTER(&htim13) - loop_timer > 4070 ){
//		  HAL_GPIO_TogglePin(led_status_GPIO_Port, led_status_Pin);
//	  }

	  while ( __HAL_TIM_GET_COUNTER(&htim13) - loop_timer < 4000 );
	  loop_timer = __HAL_TIM_GET_COUNTER(&htim13);



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if ( GPIO_Pin == GPIO_PIN_9){
		tick = __HAL_TIM_GET_COUNTER(&htim14);
		__HAL_TIM_SET_COUNTER(&htim14,0);

		if ( tick < 2100){
			ch[pulse] = tick;
			pulse++;
		}
		else{
			__HAL_TIM_SET_COUNTER(&htim14,0);
			pulse =0;
		}

	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
