/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "max30102_for_stm32_hal.h"
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum PulseStateMachine {
    PULSE_IDLE,
	PULSE_TRACE_UP,
	PULSE_TRACE_DOWN
} PulseStateMachine;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Sensor Slaves 
#define ACCEL_GYRO_SLAVE_ID		(0x00)
#define ACCEL_GYRO_ADDRESS      (0x68<<1)
// Step Threshold
#define STEP_THRESHOLD			(1.1)
// Pulse Detection Parameters
#define ALPHA                 	(0.90)
#define PULSE_MIN_THRESHOLD     (150)
#define PULSE_MAX_THRESHOLD     (2000)
#define PULSE_GO_DOWN_THRESHOLD (1)
#define PULSE_BPM_SAMPLE_SIZE   (10)
// Calorie Parameters
#define WEIGHT					(200) // in pounds
#define AGE						(25)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
HAL_StatusTypeDef I2C_Status;

/* USER CODE BEGIN PV */
struct fifo_t {
    uint32_t rawIR;
    uint32_t rawRed;
};

struct dcFilter_t {
    float w;
    float result;
};

struct meanDiffFilter_t {
    float values[10];
    uint8_t index;
    float sum;
    uint8_t count;
};               

/* MPU-9250 */
// WHO_AM_I Register
static const uint8_t WHO_AM_I =                 0x75;
// Data Registers
static const uint8_t ACCEL_XOUT_H =             0x3B;
static const uint8_t GYRO_XOUT_H =              0x43;
// Configuration register
static const uint8_t PWR_MGMT_1 =               0x6B;
static const uint8_t INT_BYPASS_CONFIG_AD =     0x37;

/* MAX30101 */
// Status Registers
static const uint8_t MAX30105_INTSTAT1 =		0x00;
static const uint8_t MAX30105_INTSTAT2 =		0x01;
static const uint8_t MAX30105_INTENABLE1 =		0x02;
static const uint8_t MAX30105_INTENABLE2 =		0x03;
// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06;
static const uint8_t MAX30105_FIFODATA =		0x07;
// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 		0x08;
static const uint8_t MAX30105_MODECONFIG = 		0x09;
static const uint8_t MAX30105_SPO2CONFIG = 	    0x0A; 
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;
// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT = 		0x1F;
static const uint8_t MAX30105_DIETEMPFRAC =     0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 	0x21;
// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 	0x30;
// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 		0xFE;
static const uint8_t MAX30105_PARTID = 			0xFF;

// Variables
uint8_t buf[100] =                          {0};
uint8_t data[500] =                         {0};
int16_t X_AXIS =                            0;
int16_t Y_AXIS =                            0;
int16_t Z_AXIS =                            0;
float X_OUTPUT =                            0.0;
float Y_OUTPUT =                            0.0;
float Z_OUTPUT =                            0.0;
uint32_t steps =                            0;
uint32_t counter =                          0;
uint8_t currentPulseDetectorState =         0;
uint32_t lastBeatThreshold =                0;
struct dcFilter_t dcFilterIR =              {0};
struct meanDiffFilter_t medianFilterIR =    {0};
float time =                                0;
max30102_t max30102; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void switchSlaveDevice(uint8_t command);
void wakeUpSensor(void);
void calculateAccelError(double *AccelXError, double *AccelYError, double *AccelZError);
void readAccelerometer(void);
void configureMAX3010X(void);
struct dcFilter_t dcRemoval(float x, float prev_w, float alpha);
float meanDiff(float M, struct meanDiffFilter_t* filterValues);
void detectPulse(float sensor_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void switchSlaveDevice(uint8_t command) {
    data[0] = command;
    /* Writing to the INT_BYPASS_CONFIG_AD register */
    I2C_Status = HAL_I2C_Mem_Write(&hi2c1, ACCEL_GYRO_ADDRESS, INT_BYPASS_CONFIG_AD, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
    if ( I2C_Status != HAL_OK ) {
        strcpy((char*)buf, "Error switching slave device over I2C!\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
    }
    else{
        strcpy((char*)buf, "Success switching slave device over I2C!\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
    }
}

void wakeUpSensor(void) {
	/* Reading the WHO_AM_I register of the Accel-Gyro Slave */
	I2C_Status = HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADDRESS, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	if ( I2C_Status != HAL_OK ) {
		strcpy((char*)buf, "Error Reading WHO_AM_I with I2C!\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
	} 
	else
	{
		strcpy((char*)buf, "Success Reading WHO_AM_I with I2C!\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
		
		if(data[0] != 0x71) {
			strcpy((char*)buf, "Invalid WHO_AM_I Register value!\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
			sprintf((char*)buf, "Actual value is: %x\r\n", data[0]);
		HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
		}
		else
		{
			strcpy((char*)buf, "Correct WHO_AM_I Register value!\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
		}
	}
	
	data[0] = 0x00;
	/* Writing to the PWR_MGMT_1 register to turn off the sleep mode */
	I2C_Status = HAL_I2C_Mem_Write(&hi2c1, ACCEL_GYRO_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	if ( I2C_Status != HAL_OK ) {
		strcpy((char*)buf, "Error Writing I2C!\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
	} 
	else {
		strcpy((char*)buf, "Success Writing I2C!\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
	}
}

void calculateAccelError(double *AccelXError, double *AccelYError, double *AccelZError) {
/* ------------------------------------------ Calculating the ERROR  ---------------------------------------- */
	uint8_t counter = 0;
	while (counter < 100)
	{
		/* --------------------------------------- ACCELEROMETER ERROR --------------------------------------------- */
		for(uint8_t i = 0; i < 6; i++) 
		{
			I2C_Status = HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADDRESS, (ACCEL_XOUT_H + i), I2C_MEMADD_SIZE_8BIT, (data + i), 1, HAL_MAX_DELAY);
			if ( I2C_Status != HAL_OK ) {
				strcpy((char*)buf, "Error Reading I2C!\r\n");
				HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
			}
		}
		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];
		
		*AccelXError += X_AXIS/16384.0;
		*AccelYError += Y_AXIS/16384.0;
		*AccelZError += Z_AXIS/16384.0;
		
		counter++;
	}
	*AccelXError = *AccelXError/100.0;
	*AccelYError = *AccelYError/100.0;
	*AccelZError = *AccelZError/100.0 - 1;
}

void readAccelerometer(void) {
	for(uint8_t i = 0; i < 6; i++) 
	{
		I2C_Status = HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADDRESS, (ACCEL_XOUT_H + i), I2C_MEMADD_SIZE_8BIT, (data + i), 1, HAL_MAX_DELAY);
		if ( I2C_Status != HAL_OK ) {
			strcpy((char*)buf, "Error reading accelerometer data with I2C!\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
		}
	}
}
void configureMAX3010X(void) {
    uint8_t en_reg[2] = {0};
	en_reg[0] = 0x40;
	max30102_write(&max30102, MAX30105_MODECONFIG, en_reg, 1);

	en_reg[0] = 0x50;
	max30102_write(&max30102, MAX30105_FIFOCONFIG, en_reg, 1);

	en_reg[0] = 0x07;
	max30102_write(&max30102, MAX30105_MODECONFIG, en_reg, 1);

	en_reg[0] = 0x20 | 0x0C | 0x03;
	max30102_write(&max30102, MAX30105_SPO2CONFIG, en_reg, 1);

	en_reg[0] = 0x1F;
	max30102_write(&max30102, MAX30105_LED1_PULSEAMP, en_reg, 1);

	en_reg[0] = 0x1F;
	max30102_write(&max30102, MAX30105_LED2_PULSEAMP, en_reg, 1);

	en_reg[0] = 0x1F;
	max30102_write(&max30102, MAX30105_LED3_PULSEAMP, en_reg, 1);

	en_reg[0] = 0x1F;
	max30102_write(&max30102, MAX30105_LED_PROX_AMP, en_reg, 1);

	en_reg[0] = 0x01 | (0x02 << 4);
	max30102_write(&max30102, MAX30105_MULTILEDCONFIG1, en_reg, 1);

	en_reg[0] = 0x00;
	max30102_write(&max30102, MAX30105_FIFOWRITEPTR, en_reg, 1);

	en_reg[0] = 0x00;
	max30102_write(&max30102, MAX30105_FIFOOVERFLOW, en_reg, 1);

	en_reg[0] = 0x00;
	max30102_write(&max30102, MAX30105_FIFOREADPTR, en_reg, 1);

	en_reg[0] = 0x0A;
    max30102_write(&max30102, MAX30105_LED1_PULSEAMP, en_reg, 1);

	en_reg[0] = 0x00;
	max30102_write(&max30102, MAX30105_LED3_PULSEAMP, en_reg, 1);
}
struct dcFilter_t dcRemoval(float x, float prev_w, float alpha) {
    struct dcFilter_t filtered;
    filtered.w = x + alpha * prev_w;
    filtered.result = filtered.w - prev_w;

    return filtered;
}
float meanDiff(float M, struct meanDiffFilter_t* filterValues) {
    float avg = 0;

    filterValues->sum -= filterValues->values[filterValues->index];
    filterValues->values[filterValues->index] = M;
    filterValues->sum += filterValues->values[filterValues->index];

    filterValues->index++;
    filterValues->index = filterValues->index % 10;

    if(filterValues->count < 10)
        filterValues->count++;

    avg = filterValues->sum / filterValues->count;
    return avg - M;
}
void detectPulse(float sensor_value) {
    static float prev_sensor_value = 0;
    static uint8_t values_went_down = 0;
    static uint32_t currentBeat = 0;
    static uint32_t lastBeat = 0;

    if(sensor_value > PULSE_MAX_THRESHOLD)
    {
        currentPulseDetectorState = PULSE_IDLE;
        prev_sensor_value = 0;
        lastBeat = 0;
        currentBeat = 0;
        values_went_down = 0;
        lastBeatThreshold = 0;
        return;
    }

    switch(currentPulseDetectorState)
    {
        case PULSE_IDLE:
            if(sensor_value >= PULSE_MIN_THRESHOLD) {
                currentPulseDetectorState = PULSE_TRACE_UP;
                values_went_down = 0;
            }
            break;

        case PULSE_TRACE_UP:
            if(sensor_value > prev_sensor_value)
            {
                currentBeat = HAL_GetTick();
                lastBeatThreshold = sensor_value;
            }
            else
            {
                sprintf((char*)buf, "Peak reached: %f\r\n", sensor_value);
                HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
                sprintf((char*)buf, "Previous value: %f\r\n", prev_sensor_value);
                HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
                counter++;
                currentPulseDetectorState = PULSE_TRACE_DOWN;    
                return;
            }
            break;

        case PULSE_TRACE_DOWN:
            if(sensor_value < prev_sensor_value)
            {
                values_went_down++;
            }

            if(sensor_value < PULSE_MIN_THRESHOLD)
            {
                currentPulseDetectorState = PULSE_IDLE;
            }
            break;
    }

    prev_sensor_value = sensor_value;
    return;
}
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
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    /* MPU-9250 */
    switchSlaveDevice(ACCEL_GYRO_SLAVE_ID);
    wakeUpSensor();

    double AccelXError = 0.0;
    double AccelYError = 0.0;
    double AccelZError = 0.0;

    calculateAccelError(&AccelXError, &AccelYError, &AccelZError);
    /* USER CODE END 2 */

    /* MAX30101 */
    max30102_init(&max30102, &hi2c1);
    max30102_reset(&max30102);
    max30102_clear_fifo(&max30102);
    configureMAX3010X();

    /* SSD1306 */
    SSD1306_Init();

    float offset = HAL_GetTick();

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
        HAL_Delay (100);
        
        /* Reading Acceleremeter Data */
        readAccelerometer();
        
        X_AXIS = (data[0] << 8) | data[1];
        Y_AXIS = (data[2] << 8) | data[3];
        Z_AXIS = (data[4] << 8) | data[5];
        
        X_OUTPUT = X_AXIS/16384.0 - AccelXError;
        Y_OUTPUT = Y_AXIS/16384.0 - AccelYError;
        Z_OUTPUT = Z_AXIS/16384.0 - AccelZError;
        
        strcpy((char*)buf, "====================ACC=============!\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "%f", X_OUTPUT);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        strcpy((char*)buf, "/");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "%f", Y_OUTPUT);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        strcpy((char*)buf, "/");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "%f\r\n", Z_OUTPUT);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        strcpy((char*)buf, "====================================!\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        
        if ((STEP_THRESHOLD*STEP_THRESHOLD) < (X_OUTPUT*X_OUTPUT + Y_OUTPUT*Y_OUTPUT + Z_OUTPUT*Z_OUTPUT)){
            steps++;
        }
        strcpy((char*)buf, "===================STEPS============!\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "%d\r\n", steps);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        strcpy((char*)buf, "====================================!\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        
        /* Reading Heart Rate Data */
        max30102_read_fifo(&max30102);
        dcFilterIR = dcRemoval((float)max30102._ir_samples[0], dcFilterIR.w, ALPHA);
        float result = meanDiff(dcFilterIR.result, &medianFilterIR);
        detectPulse(result);
        
        time = HAL_GetTick();
        time -= offset;
        time /= (60*1000);
        
        //if (time < 0.5) counter = 0;
        
        // Allow for half a minute to settle in
        float averageBPM = counter/(time - 0.2);
        if (averageBPM < 0) averageBPM = 0;
        float calories = (0.4472*averageBPM - 0.05741*WEIGHT + 0.074*AGE - 20.4022)*(time - 0.2)/4.184;
        
        /* Write data to display */
        SSD1306_GotoXY(5,2);
        sprintf((char*)buf, "Time[m]: %f", time);
        SSD1306_Puts((char*)buf, &Font_7x10, 1); 
        SSD1306_GotoXY(5,17);
        sprintf((char*)buf, "Steps: %d", steps);
        SSD1306_Puts((char*)buf, &Font_7x10, 1);
        SSD1306_GotoXY(5, 32); 
        sprintf((char*)buf, "ABPM: %f", averageBPM);
        SSD1306_Puts((char*)buf, &Font_7x10, 1);
        SSD1306_GotoXY(5, 47); 
        sprintf((char*)buf, "Calories: %f", calories);
        SSD1306_Puts((char*)buf, &Font_7x10, 1); 		
        SSD1306_UpdateScreen();
        
        strcpy((char*)buf, "===================HR===============!\r\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "IR result: %f\r\n", result);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "Counter: %d\r\n", counter);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "Average BPM: %f\r\n", averageBPM);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "Calories: %f\r\n", calories);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        sprintf((char*)buf, "Time: %f\r\n", time);
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        strcpy((char*)buf, "====================================!\r\n\n");
        HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
        
        /* USER CODE END WHILE */
        
    /* USER CODE BEGIN 3 */
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00702991;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/