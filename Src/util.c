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

// Includes
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "i2c.h"
#include "defines.h"
#include "config.h"
#include "util.h"
#include "mpu6050.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef  hi2c1;

// USART2 variables
#ifdef SERIAL_CONTROL
SerialSideboard Sideboard;
#endif

#if defined(SERIAL_DEBUG) || defined(SERIAL_FEEDBACK)
static uint8_t  rx2_buffer[SERIAL_BUFFER_SIZE]; // USART Rx DMA circular buffer
static uint32_t rx2_buffer_len = ARRAY_LEN(rx2_buffer);
#endif

#ifdef SERIAL_FEEDBACK
static SerialFeedback Feedback;
static SerialFeedback FeedbackRaw;
static uint16_t timeoutCntSerial2  = 0;         // Timeout counter for UART2 Rx Serial
static uint8_t  timeoutFlagSerial2 = 0;         // Timeout Flag for UART2 Rx Serial: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
static uint32_t Feedback_len  = sizeof(Feedback);
#endif

// USART1 variables
#ifdef SERIAL_AUX_TX
static SerialAuxTx AuxTx;
#endif

#ifdef SERIAL_AUX_RX
static uint8_t  rx1_buffer[SERIAL_BUFFER_SIZE]; // USART Rx DMA circular buffer
static uint32_t rx1_buffer_len = ARRAY_LEN(rx1_buffer);
#endif

#ifdef SERIAL_AUX_RX
static SerialCommand command;
static SerialCommand command_raw;
static uint16_t timeoutCntSerial1  = 0;         // Timeout counter for UART1 Rx Serial
static uint8_t  timeoutFlagSerial1 = 0;         // Timeout Flag for UART1 Rx Serial: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
static uint32_t command_len = sizeof(command);
extern uint8_t  print_aux;
  #ifdef CONTROL_IBUS
  static uint16_t ibus_chksum;
  static uint16_t ibus_captured_value[IBUS_NUM_CHANNELS];
  #endif
#endif

#if (defined(SERIAL_AUX_RX) && defined(CONTROL_IBUS)) || defined(SERIAL_CONTROL)
static int16_t  cmd1, cmd2;
static uint16_t cmdSwitch;
#endif

// Optical sensors variables
static GPIO_PinState sensor1, sensor2;           // holds the sensor1 and sensor 2 values
static GPIO_PinState sensor1_read, sensor2_read; // holds the instantaneous Read for sensor1 and sensor 2

// MPU variables
extern MPU_Data     mpu;                        // holds the MPU-6050 data
#if defined(MPU_SENSOR_ENABLE) || defined(SERIAL_CONTROL)
static ErrorStatus  mpuStatus;                  // holds the MPU-6050 status: SUCCESS or ERROR
#endif

extern uint32_t     main_loop_counter;          // main loop counter to perform task scheduling inside main()



/* =========================== General Functions =========================== */

void consoleLog(char *message)
{
    #ifdef SERIAL_DEBUG
        log_i("%s", message);
    #endif
}


void get_tick_count_ms(unsigned long *count)
{
    *count = HAL_GetTick();
}


/* retarget the C library printf function to the USART */
#ifdef SERIAL_DEBUG
    #ifdef __GNUC__
        #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
    #else
        #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
    #endif
    PUTCHAR_PROTOTYPE {
        HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
        return ch;
    }
    
    #ifdef __GNUC__
        int _write(int file, char *data, int len) {
            int i;
            for (i = 0; i < len; i++) { __io_putchar( *data++ );}
            return len;
        }
    #endif
#endif


void intro_demo_led(uint32_t tDelay)
{
    int i;

    for (i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
        HAL_Delay(tDelay);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_Delay(tDelay);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_Delay(tDelay);
    }

    for (i = 0; i < 2; i++) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
        HAL_Delay(tDelay);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
    }
}


uint8_t switch_check(uint16_t ch, uint8_t type) {
    if (type) { // 3 positions switch
        if      (ch < 250) return 0;    // switch in position 0
        else if (ch < 850) return 1;    // switch in position 1
        else               return 2;    // switch in position 2
    } else {    // 2 positions switch
        return  (ch > 850);
    }
}


/* =========================== Input Initialization Function =========================== */
void input_init(void) {
    #if defined(SERIAL_DEBUG) || defined(SERIAL_FEEDBACK)
        HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx2_buffer, sizeof(rx2_buffer));
        UART_DisableRxErrors(&huart2);
    #endif
    #ifdef SERIAL_AUX_RX
        HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx1_buffer, sizeof(rx1_buffer));
        UART_DisableRxErrors(&huart1);
    #endif

    intro_demo_led(100);                                                // Short LEDs intro demo with 100 ms delay. This also gives some time for the MPU-6050 to power-up.	

    #ifdef MPU_SENSOR_ENABLE
        if(mpu_config()) {                                              // IMU MPU-6050 config
            mpuStatus = ERROR;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  // Turn on RED LED - sensor enabled and NOT ok
        }
        else {
            mpuStatus = SUCCESS;
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);  // Turn on GREEN LED - sensor enabled and ok
        }
    #else
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);      // Turn on GREEN LED - sensor disabled
    #endif

    #ifdef SERIAL_DEBUG
        mpu_handle_input('h');                                          // Print the User Help commands to serial
    #endif
}

/**
  * @brief  Disable Rx Errors detection interrupts on UART peripheral (since we do not want DMA to be stopped)
  *         The incorrect data will be filtered based on the START_FRAME and checksum.
  * @param  huart: UART handle.
  * @retval None
  */
#if defined(SERIAL_DEBUG) || defined(SERIAL_FEEDBACK)
void UART_DisableRxErrors(UART_HandleTypeDef *huart)
{
  /* Disable PE (Parity Error) interrupts */
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);

  /* Disable EIE (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
}
#endif


/* =========================== Handle Functions =========================== */

/*
 * Handle of the MPU-6050 IMU sensor
 */
void handle_mpu6050(void) {
#ifdef MPU_SENSOR_ENABLE
    // Get MPU data. Because the MPU-6050 interrupt pin is not wired we have to check DMP data by pooling periodically
    if (SUCCESS == mpuStatus) {
        mpu_get_data();
    } else if (ERROR == mpuStatus && main_loop_counter % 100 == 0) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);                   // Toggle the Red LED every 100 ms
    }
    // Print MPU data to Console
    #ifdef SERIAL_DEBUG
    if (main_loop_counter % 50 == 0) {
        mpu_print_to_console();
    }
    #endif
#endif
}

/*
 * Handle of the optical sensors
 */
void handle_sensors(void) {
    sensor1_read = HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin);
    sensor2_read = HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin);

    // SENSOR1
    if (sensor1 == GPIO_PIN_RESET && sensor1_read == GPIO_PIN_SET) {
      sensor1 = GPIO_PIN_SET;
      // Sensor ACTIVE: Do something here (one time task on activation)
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
      consoleLog("SENSOR 1 ON\r\n");
    } else if(sensor1 == GPIO_PIN_SET && sensor1_read == GPIO_PIN_RESET) {
      // Sensor DEACTIVE: Do something here (one time task on deactivation)
      sensor1 = GPIO_PIN_RESET;
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
      consoleLog("SENSOR 1 OFF\r\n");
    }

    // SENSOR2
    if (sensor2 == GPIO_PIN_RESET && sensor2_read == GPIO_PIN_SET) {
      sensor2 = GPIO_PIN_SET;
      // Sensor ACTIVE: Do something here (one time task on activation)
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
      consoleLog("SENSOR 2 ON\r\n");
    } else if (sensor2 == GPIO_PIN_SET && sensor2_read == GPIO_PIN_RESET) {
      // Sensor DEACTIVE: Do something here (one time task on deactivation)
      sensor2 = GPIO_PIN_RESET;
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
      consoleLog("SENSOR 2 OFF\r\n");
    }

    if (sensor1 == GPIO_PIN_SET) {
      // Sensor ACTIVE: Do something here (continuous task)
    }
    if (sensor2 == GPIO_PIN_SET) {
      // Sensor ACTIVE: Do something here (continuous task)
    }
}

/*
 * Handle of the USART data
 */
void handle_usart(void) {
    // Tx USART MAIN
    #ifdef SERIAL_CONTROL
        if (main_loop_counter % 5 == 0 && __HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {     // Check if DMA channel counter is 0 (meaning all data has been transferred)
            Sideboard.start     = (uint16_t)SERIAL_START_FRAME;
            Sideboard.pitch     = (int16_t)mpu.euler.pitch;
            Sideboard.dPitch    = (int16_t)mpu.gyro.y;
            Sideboard.cmd1      = (int16_t)cmd1;
            Sideboard.cmd2      = (int16_t)cmd2; 
            Sideboard.sensors   = (uint16_t)( (cmdSwitch << 8)  | (sensor1 | (sensor2 << 1) | (mpuStatus << 2)) );
            Sideboard.checksum  = (uint16_t)(Sideboard.start ^ Sideboard.pitch ^ Sideboard.dPitch ^ Sideboard.cmd1 ^ Sideboard.cmd2 ^ Sideboard.sensors);

            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Sideboard, sizeof(Sideboard));
        }
    #endif
    // Rx USART MAIN
    #ifdef SERIAL_FEEDBACK
        if (timeoutCntSerial2++ >= SERIAL_TIMEOUT) {                // Timeout qualification
            timeoutFlagSerial2 = 1;                                 // Timeout detected
            timeoutCntSerial2  = SERIAL_TIMEOUT;                    // Limit timout counter value
        }
        if (timeoutFlagSerial2 && main_loop_counter % 100 == 0) {   // In case of timeout bring the system to a Safe State and indicate error if desired
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);           // Toggle the Yellow LED every 100 ms
        }
    #endif

    // Tx USART AUX
    #ifdef SERIAL_AUX_TX
        if (main_loop_counter % 5 == 0 && __HAL_DMA_GET_COUNTER(huart1.hdmatx) == 0) {     // Check if DMA channel counter is 0 (meaning all data has been transferred)
            AuxTx.start     = (uint16_t)SERIAL_START_FRAME;
            AuxTx.signal1   = (int16_t)sensor1;
            AuxTx.signal2   = (int16_t)sensor2;
            AuxTx.checksum  = (uint16_t)(AuxTx.start ^ AuxTx.signal1 ^ AuxTx.signal2);

            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&AuxTx, sizeof(AuxTx));
        }
    #endif
    // Rx USART AUX
    #ifdef SERIAL_AUX_RX
        #ifdef CONTROL_IBUS
        if (!timeoutFlagSerial1) {
            for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i+=2) {
                ibus_captured_value[(i/2)] = CLAMP(command.channels[i] + (command.channels[i+1] << 8) - 1000, 0, 1000); // 1000-2000 -> 0-1000
            }
            cmd1        = (ibus_captured_value[0] - 500) * 2;                           // Channel 1
            cmd2        = (ibus_captured_value[1] - 500) * 2;                           // Channel 2
            cmdSwitch   = (uint16_t)(switch_check(ibus_captured_value[6],0)      |      // Channel 7
                                     switch_check(ibus_captured_value[7],1) << 1 |      // Channel 8
                                     switch_check(ibus_captured_value[8],1) << 3 |      // Channel 9
                                     switch_check(ibus_captured_value[9],0) << 5);      // Channel 10
        }
        #endif

        if (timeoutCntSerial1++ >= SERIAL_TIMEOUT) {                // Timeout qualification
            timeoutFlagSerial1 = 1;                                 // Timeout detected
            timeoutCntSerial1  = SERIAL_TIMEOUT;                    // Limit timout counter value
            cmd1 = cmd2 = 0;                                        // Set commands to 0
            cmdSwitch &= ~(1U << 0);                                // Clear Bit 0, to switch to default control input
        }
        // if (timeoutFlagSerial0 && main_loop_counter % 100 == 0) {   // In case of timeout bring the system to a Safe State and indicate error if desired
        //     HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);           // Toggle the Green LED every 100 ms
        // }

        #ifdef SERIAL_DEBUG
            // Print MPU data to Console
            if (main_loop_counter % 50 == 0) {
                aux_print_to_console();
            }
        #endif
    #endif
}

/*
 * Handle of the sideboard LEDs
 */
void handle_leds(void) {
    #ifdef SERIAL_FEEDBACK
      if (!timeoutFlagSerial2) {
        if (Feedback.cmdLed & LED1_SET) { HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); }
        if (Feedback.cmdLed & LED2_SET) { HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); }
        if (Feedback.cmdLed & LED3_SET) { HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); }
        if (Feedback.cmdLed & LED4_SET) { HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET); }
        if (Feedback.cmdLed & LED5_SET) { HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET); } else { HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET); }
      }
    #endif
}


/* =========================== USART2 READ Functions =========================== */

/*
 * Check for new data received on USART with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart2_rx_check(void)
{
    #ifdef SERIAL_DEBUG
    static uint32_t old_pos;
    uint32_t pos;

    pos = rx2_buffer_len - __HAL_DMA_GET_COUNTER(huart2.hdmarx);                // Calculate current position in buffer, Rx: DMA1_Channel6->CNDTR, Tx: DMA1_Channel7
    if (pos != old_pos) {                                                       // Check change in received data
        if (pos > old_pos) {                                                    // "Linear" buffer mode: check if current position is over previous one
            usart_process_debug(&rx2_buffer[old_pos], pos - old_pos);           // Process data
        } else {                                                                // "Overflow" buffer mode
             usart_process_debug(&rx2_buffer[old_pos], rx2_buffer_len - old_pos);// First Process data from the end of buffer
            if (pos > 0) {                                                      // Check and continue with beginning of buffer
                usart_process_debug(&rx2_buffer[0], pos);                       // Process remaining data
            }
        }
    }
    old_pos = pos;                                                              // Updated old position
    if (old_pos == rx2_buffer_len) {                                            // Check and manually update if we reached end of buffer
        old_pos = 0;
    }
    #endif // SERIAL_DEBUG

    #ifdef SERIAL_FEEDBACK
    static uint32_t old_pos;
    uint32_t pos;
    uint8_t *ptr;
    
    pos = rx2_buffer_len - __HAL_DMA_GET_COUNTER(huart2.hdmarx);                // Calculate current position in buffer, Rx: DMA1_Channel6->CNDTR, Tx: DMA1_Channel7
    if (pos != old_pos) {                                                       // Check change in received data
        ptr = (uint8_t *)&FeedbackRaw;                                          // Initialize the pointer with FeedbackRaw address
        if (pos > old_pos && (pos - old_pos) == Feedback_len) {                 // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
            memcpy(ptr, &rx2_buffer[old_pos], Feedback_len);                    // Copy data. This is possible only if FeedbackRaw is contiguous! (meaning all the structure members have the same size)
            usart_process_data(&FeedbackRaw, &Feedback);                        // Process data
        } else if ((rx2_buffer_len - old_pos + pos) == Feedback_len) {          // "Overflow" buffer mode: check if data length equals expected length
            memcpy(ptr, &rx2_buffer[old_pos], rx2_buffer_len - old_pos);        // First copy data from the end of buffer
            if (pos > 0) {                                                      // Check and continue with beginning of buffer
                ptr += rx2_buffer_len - old_pos;                                // Move to correct position in FeedbackRaw
                memcpy(ptr, &rx2_buffer[0], pos);                               // Copy remaining data
            }
            usart_process_data(&FeedbackRaw, &Feedback);                        // Process data
        }
    }
    old_pos = pos;                                                              // Update old position
    if (old_pos == rx2_buffer_len) {                                            // Check and manually update if we reached end of buffer
        old_pos = 0;
    }
    #endif // SERIAL_FEEDBACK
}

/*
 * Process Rx debug user command input
 */
#ifdef SERIAL_DEBUG
void usart_process_debug(uint8_t *userCommand, uint32_t len)
{
    for (; len > 0; len--, userCommand++) {
        if (*userCommand != '\n' && *userCommand != '\r') {     // Do not accept 'new line' and 'carriage return' commands
            log_i("Command = %c\r\n", *userCommand);
            mpu_handle_input(*userCommand);
        }
    }
}
#endif // SERIAL_DEBUG

/*
 * Process Rx data
 * - if the Feedback_in data is valid (correct START_FRAME and checksum) copy the Feedback_in to Feedback_out
 */
#ifdef SERIAL_FEEDBACK
void usart_process_data(SerialFeedback *Feedback_in, SerialFeedback *Feedback_out)
{
    uint16_t checksum;
    if (Feedback_in->start == SERIAL_START_FRAME) {
        checksum = (uint16_t)(Feedback_in->start ^ Feedback_in->cmd1 ^ Feedback_in->cmd2 ^ Feedback_in->speedR_meas ^ Feedback_in->speedL_meas
                            ^ Feedback_in->batVoltage ^ Feedback_in->boardTemp ^ Feedback_in->cmdLed);
        if (Feedback_in->checksum == checksum) {
            *Feedback_out = *Feedback_in;
            timeoutCntSerial2  = 0;     // Reset timeout counter
            timeoutFlagSerial2 = 0;     // Clear timeout flag
        }
    }
}
#endif // SERIAL_FEEDBACK


/* =========================== USART1 READ Functions =========================== */

/*
 * Check for new data received on USART with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart1_rx_check(void)
{
    #ifdef SERIAL_AUX_RX
    static uint32_t old_pos;
    uint32_t pos;
    uint8_t *ptr;

    pos = rx1_buffer_len - __HAL_DMA_GET_COUNTER(huart1.hdmarx);                // Calculate current position in buffer, Rx: DMA1_Channel5->CNDTR, Tx: DMA1_Channel4
    if (pos != old_pos) {                                                       // Check change in received data
        ptr = (uint8_t *)&command_raw;                                          // Initialize the pointer with structure address
        if (pos > old_pos && (pos - old_pos) == command_len) {                  // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
            memcpy(ptr, &rx1_buffer[old_pos], command_len);                     // Copy data. This is possible only if structure is contiguous! (meaning all the structure members have the same size)
            usart_process_command(&command_raw, &command);                      // Process data
        } else if ((rx1_buffer_len - old_pos + pos) == command_len) {           // "Overflow" buffer mode: check if data length equals expected length
            memcpy(ptr, &rx1_buffer[old_pos], rx1_buffer_len - old_pos);        // First copy data from the end of buffer
            if (pos > 0) {                                                      // Check and continue with beginning of buffer
                ptr += rx1_buffer_len - old_pos;                                // Update position
                memcpy(ptr, &rx1_buffer[0], pos);                               // Copy remaining data
            }
            usart_process_command(&command_raw, &command);                      // Process data
        }
    }
    old_pos = pos;                                                              // Updated old position
    if (old_pos == rx1_buffer_len) {                                            // Check and manually update if we reached end of buffer
        old_pos = 0;
    }
    #endif  // SERIAL_AUX_RX
}

/*
 * Process command UART0 Rx data
 * - if the command_in data is valid (correct START_FRAME and checksum) copy the command_in to command_out
 */
#ifdef SERIAL_AUX_RX
void usart_process_command(SerialCommand *command_in, SerialCommand *command_out)
{
  #ifdef CONTROL_IBUS
    if (command_in->start == IBUS_LENGTH && command_in->type == IBUS_COMMAND) {
      ibus_chksum = 0xFFFF - IBUS_LENGTH - IBUS_COMMAND;
      for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i++) {
        ibus_chksum -= command_in->channels[i];
      }
      if (ibus_chksum == (uint16_t)((command_in->checksumh << 8) + command_in->checksuml)) {
        *command_out = *command_in;
        timeoutCntSerial1  = 0;        // Reset timeout counter
        timeoutFlagSerial1 = 0;        // Clear timeout flag
      }
    }
  #endif
}
#endif


/* =========================== AUX Serial Print data =========================== */

void aux_print_to_console(void)
{
#if defined(SERIAL_DEBUG) && defined(SERIAL_AUX_RX)
    #ifdef CONTROL_IBUS
    if (print_aux & PRINT_AUX) {
        log_i( "Ch1: %d Ch2: %d Sw: %u\r\n", cmd1, cmd2, cmdSwitch);
    }
    #endif
#endif
}




/* =========================== I2C WRITE Functions =========================== */

/*
 * write bytes to chip register
 */
int8_t i2c_writeBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{	
    // !! Using the I2C Interrupt will fail writing the DMP.. could be that DMP memory writing requires more time !! So use the I2C without interrupt.
    // HAL_I2C_Mem_Write_IT(&hi2c1, slaveAddr << 1, regAddr, 1, data, length);
    // while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));                     // Wait until all data bytes are sent/received
    // return 0;
#ifdef I2C_SINGLE_BYTE
    HAL_StatusTypeDef res = HAL_OK;
    if(regAddr==0x6b && length ==2){
        uint8_t i;
        for(i=0; i < length && res == HAL_OK; i++) {
            res = HAL_I2C_Mem_Write(&hi2c1, slaveAddr << 1, regAddr+i, 1, data+i, 1, 100);    // Address is shifted one position to the left. LSB is reserved for the Read/Write bit.
        }
    }
      else {
            res = HAL_I2C_Mem_Write(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100);    // Address is shifted one position to the left. LSB is reserved for the Read/Write bit. 
      }
      if(res != HAL_OK) {
         printf("i2c_writeBytes not ok %u %u\r\n", regAddr, length);
      }
      return res;
#else
    return HAL_I2C_Mem_Write(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100);    // Address is shifted one position to the left. LSB is reserved for the Read/Write bit.
#endif
}

/*
 * write 1 byte to chip register
 */
int8_t i2c_writeByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    return i2c_writeBytes(slaveAddr, regAddr, 1, &data);
}

/*
 * write one bit to chip register
 */
int8_t i2c_writeBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2c_readByte(slaveAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2c_writeByte(slaveAddr, regAddr, b);
}

/* =========================== I2C READ Functions =========================== */

/*
 * read bytes from chip register
 */
int8_t i2c_readBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    // !! Using the I2C Interrupt will fail writing the DMP.. could be that DMP memory writing requires more time !! So use the I2C without interrupt.
    // HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr, 1, data, length);
    // while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));                   // Wait until all data bytes are sent/received
    // return 0;
#ifdef I2C_SINGLE_BYTE
    HAL_StatusTypeDef res = HAL_OK;
    if((regAddr==0x72 || regAddr == 0x41) && length ==2) {
       uint8_t i;
       for(i=0; i < length && res == HAL_OK; i++) {
          res= HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr+i, 1, data+i, 1, 100);     // Address is shifted one position to the left. LSB is reserved for the Read/Write bit.
       }
    }
    else {
        res = HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100);     // Address is shifted one position to the left. LSB is reserved for the Read/Write bit.

    }
      if(res != HAL_OK) {
        printf("i2c_readBytes not ok %u %u\r\n", regAddr, length);
      }
      return res;
#else
    return HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100);     // Address is shifted one position to the left. LSB is reserved for the Read/Write bit.
#endif

//    

}

/*
 * read 1 byte from chip register
 */
int8_t i2c_readByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data)
{
    return i2c_readBytes(slaveAddr, regAddr, 1, data);
}

/*
 * read 1 bit from chip register
 */
int8_t i2c_readBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t b;
    int8_t status = i2c_readByte(slaveAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return status;
}




#ifdef USE_RCC_DELAY
void HAL_Delay(uint32_t mdelay)
{
  __IO uint32_t Delay = mdelay * (SystemCoreClock / 8U / 1000U);
  do
  {
    __NOP();
  }
  while (Delay --);
}
#endif



