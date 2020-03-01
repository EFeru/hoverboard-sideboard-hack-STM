/* USER CODE BEGIN Header */
/**
  * This file is part of the hoverboard-sideboard-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "defines.h"
#include "config.h"
#include "util.h"
#include "mpu6050.h"
#include "mpu6050_dmp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart2;
uint8_t rxBuffer, userCommand = 0;							// holds the user command input

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_FLUSH_DRREGISTER(&huart2);           // Clear the buffer to prevent overrun
    
  #ifdef SERIAL_DEBUG  
    if (rxBuffer != '\n' && rxBuffer != '\r') { 	// Do not accept 'new line' (ascii 10) and 'carriage return' (ascii 13) commands
        userCommand = rxBuffer;
    }
  #endif
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef SERIAL_CONTROL
typedef struct{
  uint16_t	start;
  int16_t  	roll;
  int16_t  	pitch;
  int16_t  	yaw;
  uint16_t  sensors;
  uint16_t 	checksum;
} SerialSideboard;
SerialSideboard Sideboard;
#endif

#ifdef SERIAL_FEEDBACK
typedef struct{
	uint16_t 	start;
	int16_t 	cmd1;
	int16_t 	cmd2;
	int16_t 	speedR_meas;
	int16_t 	speedL_meas;
	int16_t 	batVoltage;
	int16_t 	boardTemp;
	uint16_t 	cmdLed;
	uint16_t  checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

static int16_t timeoutCntSerial   = 0;  				// Timeout counter for Rx Serial command
static uint8_t timeoutFlagSerial  = 0;  				// Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
#endif

extern MPU_Data 	mpu;													// holds the MPU-6050 data
ErrorStatus				mpuStatus;					          // holds the MPU-6050 status: SUCCESS or ERROR

GPIO_PinState	  	sensor1, sensor2; 					  // holds the sensor1 and sensor 2 values
GPIO_PinState	  	sensor1_read, sensor2_read;	  // holds the instantaneous Read for sensor1 and sensor 2

static uint32_t 	main_loop_counter;						// main loop counter to perform task squeduling inside main()
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  #ifdef SERIAL_DEBUG
    __HAL_UART_FLUSH_DRREGISTER(&huart2);
    HAL_UART_Receive_DMA (&huart2, (uint8_t *)&rxBuffer, sizeof(rxBuffer));
  #endif
  #ifdef SERIAL_CONTROL
    __HAL_UART_FLUSH_DRREGISTER(&huart2);
  	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Sideboard, sizeof(Sideboard));
  #endif
  #ifdef SERIAL_FEEDBACK
    __HAL_UART_FLUSH_DRREGISTER(&huart2);
		HAL_UART_Receive_DMA (&huart2, (uint8_t *)&NewFeedback, sizeof(NewFeedback));
	#endif

	intro_demo_led(100);								// Short LEDs intro demo with 100 ms delay. This also gives some time for the MPU-6050 to power-up.	
	if(mpu_config()) { 									// IMU MPU-6050 config
		mpuStatus = ERROR;
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);  // Turn on RED LED
	}
	else {
		mpuStatus = SUCCESS;
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);  // Turn on GREEN LED
	}
	mpu_handle_input('h'); 						  // Print the User Help commands to serial

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {    
    HAL_Delay(DELAY_IN_MAIN_LOOP);

    // ==================================== LEDs Handling ====================================
		// HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);                // Toggle BLUE1 LED
		#ifdef SERIAL_FEEDBACK
			if (!timeoutFlagSerial) {
				if (Feedback.cmdLed & LED1_SET) { HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); }
				if (Feedback.cmdLed & LED2_SET) { HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); }
				if (Feedback.cmdLed & LED3_SET) { HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); }
				if (Feedback.cmdLed & LED4_SET) { HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET); }
				if (Feedback.cmdLed & LED5_SET) { HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET); }
			}
		#endif

		// ==================================== USER Handling ====================================
		#ifdef SERIAL_DEBUG
      // Get the user Input as one character from Serial
      if (userCommand != 0) { 					// Check the availability of a user command set by the UART DMA
        log_i("Command = %c\n", userCommand);						
        mpu_handle_input(userCommand);
        userCommand = 0;
      }
		#endif			
      
		
		// ==================================== MPU-6050 Handling ====================================
		// Get MPU data. Because the MPU-6050 interrupt pin is not wired we have to check DMP data by pooling periodically
		if (SUCCESS == mpuStatus) {
			mpu_get_data();
		} else if (ERROR == mpuStatus && main_loop_counter % 100 == 0) {
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);				 	// Toggle the Red LED every 100 ms
		}
		// Print MPU data to Console
		if (main_loop_counter % 50 == 0) {
			mpu_print_to_console();
		}	
		

		// ==================================== SENSORS Handling ====================================
    sensor1_read = HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin);
		sensor2_read = HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin);

		// SENSOR1
		if (sensor1 == GPIO_PIN_RESET && sensor1_read == GPIO_PIN_SET) {
      sensor1 = GPIO_PIN_SET;
			// Sensor ACTIVE: Do something here (one time task on activation)
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			consoleLog("-- SENSOR 1 Active --\n");		
		} else if(sensor1 == GPIO_PIN_SET && sensor1_read == GPIO_PIN_RESET) {
      sensor1 = GPIO_PIN_RESET;
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);			
		}
		
		// SENSOR2
		if (sensor2 == GPIO_PIN_RESET && sensor2_read == GPIO_PIN_SET) {
      sensor2 = GPIO_PIN_SET;
			// Sensor ACTIVE: Do something here (one time task on activation)
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
			consoleLog("-- SENSOR 2 Active --\n");
		} else if (sensor2 == GPIO_PIN_SET && sensor2_read == GPIO_PIN_RESET) {
      sensor2 = GPIO_PIN_RESET;
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		}

    if (sensor1 == GPIO_PIN_SET) {
      // Sensor ACTIVE: Do something here (continuous task)
		}
		if (sensor2 == GPIO_PIN_SET) {
			// Sensor ACTIVE: Do something here (continuous task)
		}
		
		
		// ==================================== SERIAL Tx/Rx Handling ====================================
		#ifdef SERIAL_CONTROL						
      if (main_loop_counter % 5 == 0) {		//  Transmit Tx data periodically using DMA
				Sideboard.start    	= (uint16_t)SERIAL_START_FRAME;
				Sideboard.roll    	= (int16_t)mpu.euler.roll;
				Sideboard.pitch    	= (int16_t)mpu.euler.pitch;
				Sideboard.yaw    		= (int16_t)mpu.euler.yaw;
				Sideboard.sensors		= (uint16_t)(sensor1 | (sensor2 << 1) | (mpuStatus << 2));
				Sideboard.checksum 	= (uint16_t)(Sideboard.start ^ Sideboard.roll ^ Sideboard.pitch ^ Sideboard.yaw ^ Sideboard.sensors);
			
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Sideboard, sizeof(Sideboard));	
			}
		#endif
		
		#ifdef SERIAL_FEEDBACK
      uint16_t checksum;
      checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
      if (NewFeedback.start == SERIAL_START_FRAME && NewFeedback.checksum == checksum) {
          if (timeoutFlagSerial) {                    // Check for previous timeout flag  
            if (timeoutCntSerial-- <= 0)              // Timeout de-qualification
              timeoutFlagSerial = 0;                  // Timeout flag cleared           
          } else {
            memcpy(&Feedback, &NewFeedback, sizeof(Feedback));	// Copy the new data 
            NewFeedback.start = 0xFFFF;               // Change the Start Frame for timeout detection in the next cycle
            timeoutCntSerial  = 0;                    // Reset the timeout counter         
          }
      } else {
        if (timeoutCntSerial++ >= SERIAL_TIMEOUT) {   // Timeout qualification            
          timeoutFlagSerial = 1;                      // Timeout detected
          timeoutCntSerial  = SERIAL_TIMEOUT;         // Limit timout counter value
        }
        // Most probably we are out-of-sync. Try to re-sync by reseting the DMA
        if (main_loop_counter % 150 == 0) {
          HAL_UART_DMAStop(&huart2);
          HAL_UART_Receive_DMA(&huart2, (uint8_t *)&NewFeedback, sizeof(NewFeedback));
        }
      }
            
      if (timeoutFlagSerial && main_loop_counter % 100 == 0) {        // In case of timeout bring the system to a Safe State and indicate error if desired
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);					        // Toggle the Yellow LED every 100 ms
      }
		#endif	

    main_loop_counter++;
    
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
