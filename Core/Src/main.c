/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor_types.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "stm32c0xx_hal_flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Old structure removed - now using BLDM sensor format with float values

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Old frame validation constants removed - now using BLDM sensor format
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UART_RESPONSE_NO_ERROR (0x00)
#define I2C_CMD_ZERO_SENSOR     (0x01)  // I2C command byte to trigger zero sensor calibration
#define I2C_CMD_SPAN_SENSOR     (0x02)  // I2C command byte to trigger span sensor calibration
#define I2C_CMD_ZERO_CALCCONC   (0x05)  // I2C command: zero CalcConc (adjust coeff_c to zero CalcConc, stored in Flash)
#define CAL_STATUS_INVALID_MARKER 0xFF  // Marker value to indicate calibration status is not valid (display as "xxxx")

// Flash storage for coeff_c (stored in last Flash page)
// STM32C0 requires 8-byte (double word) alignment for Flash programming
#define FLASH_COEFF_C_MAGIC     0x434F4546UL  // Magic number: "COEF" in ASCII
#define FLASH_COEFF_C_PAGE      15U           // Use last page (page 15) for storage
#define FLASH_COEFF_C_ADDRESS   (FLASH_BASE + (FLASH_COEFF_C_PAGE * FLASH_PAGE_SIZE))  // Address in last page (must be 8-byte aligned)
#define FLASH_COEFF_C_MAGIC_OFFSET  0   // Magic number at offset 0 (first 4 bytes)
#define FLASH_COEFF_C_VALUE_OFFSET  4   // coeff_c float value at offset 4 (next 4 bytes)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t  uart_rx_buffer[32] = { 0u };  // Buffer for BLDM sensor response (23 bytes)
uint8_t  i2c_slave_cmd_expected_length = 0xFE;
uint8_t  i2c_slave_rx_buffer[64] = { 0u };
uint32_t i2c_errors = 0uL;
static uint8_t i2cTxBuf[19];  // 19 data bytes (BoardID[2] + Concentration + Temperature + Humidity + Absorptivity + ADC + Calibration Status Byte7 + Calibration Status Byte8 + CalcConc)
static uint8_t i2cDummyRx;
static volatile uint8_t i2c_data_ready = 0u;
static uint8_t i2c_cmd_byte = 0u;  // Buffer to receive I2C write command byte
static volatile uint8_t i2c_busy = 0u;  // Flag to prevent BuildI2CTxBuf during I2C operations
static volatile uint8_t zero_sensor_pending = 0u;  // Flag to indicate zero sensor command needs to be processed in main loop
static volatile uint8_t span_sensor_pending = 0u;  // Flag to indicate span sensor command needs to be processed in main loop
static volatile uint8_t zero_calc_conc_pending = 0u;  // Flag: Zero CalcConc command received, process in main loop
static int32_t last_calc_conc_avg = 0;  // Last CalcConc written to I2C buffer; used when Zero CalcConc runs (signed 32-bit)
static float coeff_c_rw;  // Runtime coeff_c; init from Flash (or default if Flash is empty); Zero CalcConc adjusts and saves to Flash
static uint8_t span_value_bytes[4] = { 0u };  // Buffer to receive 4-byte span value from I2C master
static volatile uint8_t waiting_for_span_bytes = 0u;  // Flag to indicate we're waiting for span data bytes
static uint32_t polling_suspend_until = 0uL;  // Timestamp when polling should resume (0 = not suspended)
static uint8_t calibration_response_buffer[32] = { 0u };  // Buffer to store calibration command response from sensor
static uint8_t calibration_response_length = 0u;  // Length of calibration response received
static volatile uint8_t calibration_response_ready = 0u;  // Flag indicating calibration response is available
static uint8_t calibration_status_byte7 = CAL_STATUS_INVALID_MARKER;  // 7th byte from Command 3 or 6 response (status byte 1)
static uint8_t calibration_status_byte8 = CAL_STATUS_INVALID_MARKER;  // 8th byte from Command 3 or 6 response (status byte 2)
static volatile uint8_t calibration_status_valid = 0u;  // Flag indicating calibration status bytes are valid


// Zero sensor command sequence - six commands with delays
// Command 1: FB 68 08 00 00 60 00 80 98 31 FC
static const uint8_t zero_sensor_uart_cmd_1[] = { 0xFB, 0x68, 0x08, 0x00, 0x00, 0x60, 0x00, 0x80, 0x98, 0x31, 0xFC };
// Command 2: FB 68 08 00 00 60 00 00 1B 32 FC
static const uint8_t zero_sensor_uart_cmd_2[] = { 0xFB, 0x68, 0x08, 0x00, 0x00, 0x60, 0x00, 0x00, 0x1B, 0x32, 0xFC };
// Command 3: FB 68 08 00 00 60 00 82 18 3E FC
static const uint8_t zero_sensor_uart_cmd_3[] = { 0xFB, 0x68, 0x08, 0x00, 0x00, 0x60, 0x00, 0x82, 0x18, 0x3E, 0xFC };
// Command 4: FB 68 08 00 00 60 01 80 1E 32 FC
static const uint8_t zero_sensor_uart_cmd_4[] = { 0xFB, 0x68, 0x08, 0x00, 0x00, 0x60, 0x01, 0x80, 0x1E, 0x32, 0xFC };
// Command 5: FB 68 08 00 00 60 01 00 9D 31 FC
static const uint8_t zero_sensor_uart_cmd_5[] = { 0xFB, 0x68, 0x08, 0x00, 0x00, 0x60, 0x01, 0x00, 0x9D, 0x31, 0xFC };
// Command 6: FB 68 08 00 00 60 01 82 9E 3D FC
static const uint8_t zero_sensor_uart_cmd_6[] = { 0xFB, 0x68, 0x08, 0x00, 0x00, 0x60, 0x01, 0x82, 0x9E, 0x3D, 0xFC };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t read_adc12_once(void);
static void BuildI2CTxBuf(void);
static int32_t calculate_calc_conc(float absorptivity);
static uint16_t crc16_custom(const uint8_t *data, uint16_t len);
static void send_span_sensor_command(const uint8_t *span_bytes);
static uint8_t receive_calibration_response(void);
static void send_zero_sensor_sequence(void);
static uint8_t read_coeff_c_from_flash(float *coeff_c);
static uint8_t write_coeff_c_to_flash(float coeff_c);

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  (void)AddrMatchCode;
  if (hi2c->Instance != I2C1) {
    return;
  }
  
  /* New START = new transaction: clear any stuck state from a previous incomplete transaction
   * (code otherwise only clears i2c_busy on ListenCplt or Error; one incomplete transfer could leave it stuck) */
  i2c_busy = 0u;
  i2c_busy = 1u;  // Set flag to prevent BuildI2CTxBuf from running during this transaction
  
  /* When master reads from slave, transmit buffer */
  if (TransferDirection == I2C_DIRECTION_RECEIVE) {
      /* Normal sensor data - transmit pre-built buffer (19 bytes: 2 board ID + 12 sensor data + 2 calibration status + 4 CalcConc) */
      if (HAL_OK != HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2cTxBuf, sizeof(i2cTxBuf), I2C_LAST_FRAME)) {
          i2c_busy = 0u;
          i2c_errors++;  // Diagnostic: track transmit errors
          Error_Handler();
      }
  } else {
    /* Master writing to us - could be:
     * 1. I2C scan (0-1 bytes, then STOP immediately)
     * 2. Zero sensor command (1 byte: 0x01)
     * 3. Span sensor command (1 byte: 0x02 + 4 data bytes)
     */
    /* Clear state flags for new transaction */
    waiting_for_span_bytes = 0u;
    i2c_cmd_byte = 0u;  // Clear command byte
    
    /* Receive command byte into buffer - use FIRST_FRAME in case more bytes follow */
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_cmd_byte, 1, I2C_FIRST_FRAME);
    /* Don't check return value here - during scan master may send STOP immediately after address */
    /* Command byte will be processed in HAL_I2C_SlaveRxCpltCallback */
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    i2c_busy = 0u;  // Clear flag - I2C transaction complete
    
    /* Restart listening to handle next transaction */
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

/* Callback when slave receive completes - check for zero sensor or span sensor command */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) {
    return;
  }
  
  /* Check if we're waiting for span data bytes */
  if (waiting_for_span_bytes) {
    /* The 4 span data bytes have been received - set flag for main loop to process */
    waiting_for_span_bytes = 0u;  // Clear waiting flag
    span_sensor_pending = 1u;     // Set flag to process span command in main loop
    i2c_cmd_byte = 0u;             // Clear command byte
    return;
  }
  
  /* First byte received - check command type */
  if (i2c_cmd_byte == I2C_CMD_ZERO_SENSOR) {
    /* Zero sensor command detected (0x01) - set flag for main loop to handle */
    /* CRITICAL: Do NOT do blocking UART operations here - it interferes with I2C state machine */
    /* Defer UART operations to main loop to avoid blocking I2C transaction completion */
    zero_sensor_pending = 1u;
    i2c_cmd_byte = 0u;  // Clear command byte
  } else if (i2c_cmd_byte == I2C_CMD_SPAN_SENSOR) {
    /* Span sensor command detected (0x02) - need to receive 4 data bytes */
    waiting_for_span_bytes = 1u;  // Set flag to indicate we're waiting for data bytes
    /* Initiate receive of 4 data bytes into span_value_bytes buffer */
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, span_value_bytes, 4, I2C_LAST_FRAME);
    /* Don't clear i2c_cmd_byte yet - we'll clear it after data bytes arrive */
  } else if (i2c_cmd_byte == I2C_CMD_ZERO_CALCCONC) {
    /* Zero CalcConc (0x05): subtract current CalcConc from coeff_c; defer to main loop */
    zero_calc_conc_pending = 1u;
    i2c_cmd_byte = 0u;
  } else {
    /* Unknown command or I2C scan - ignore it */
    i2c_cmd_byte = 0u;  // Clear command byte
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    i2c_busy = 0u;  // Clear flag on error to allow recovery
    i2c_errors++;   // Diagnostic: track errors
    /* Recover by re-enabling listen */
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

static uint16_t read_adc12_once(void)
{
  uint16_t value = 0;
  if (HAL_ADC_Start(&hadc1) == HAL_OK) {
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
      value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
  }
  return value;
}

// Read coeff_c from Flash memory
// Returns 1 if valid data found, 0 if Flash is empty/invalid (use default)
static uint8_t read_coeff_c_from_flash(float *coeff_c)
{
  uint32_t magic = *(volatile uint32_t*)(FLASH_COEFF_C_ADDRESS + FLASH_COEFF_C_MAGIC_OFFSET);
  float stored_value = *(volatile float*)(FLASH_COEFF_C_ADDRESS + FLASH_COEFF_C_VALUE_OFFSET);
  
  // Check if magic number matches (valid data)
  if (magic == FLASH_COEFF_C_MAGIC) {
    *coeff_c = stored_value;
    return 1u;  // Valid data found
  }
  
  return 0u;  // Flash is empty or invalid, use default
}

// Write coeff_c to Flash memory
// Returns 1 on success, 0 on failure
// Note: STM32C0 requires 8-byte (double word) programming
static uint8_t write_coeff_c_to_flash(float coeff_c)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;
  HAL_StatusTypeDef status;
  uint32_t magic = FLASH_COEFF_C_MAGIC;
  uint32_t value_as_uint32;
  uint64_t double_word_data;
  
  // Unlock Flash
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return 0u;
  }
  
  // Erase the page containing coeff_c
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page = FLASH_COEFF_C_PAGE;
  EraseInitStruct.NbPages = 1;
  
  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    return 0u;
  }
  
  // Combine magic number and coeff_c into a 64-bit double word
  // Lower 32 bits: magic number, Upper 32 bits: coeff_c float value
  memcpy(&value_as_uint32, &coeff_c, sizeof(float));
  double_word_data = ((uint64_t)value_as_uint32 << 32) | (uint64_t)magic;
  
  // Program as double word (8 bytes) at 8-byte aligned address
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              FLASH_COEFF_C_ADDRESS + FLASH_COEFF_C_MAGIC_OFFSET, 
                              double_word_data);
  
  // Lock Flash
  HAL_FLASH_Lock();
  
  return (status == HAL_OK) ? 1u : 0u;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0: helpers */
// BLDM sensor checksum calculation (sum16)
static uint16_t bldm_sum16(const uint8_t *data, uint16_t len)
{
  uint32_t sum = 0;
  for (uint16_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return (uint16_t)(sum & 0xFFFF);
}

// Build BLDM simple read request (17 bytes total)
static void bldm_build_request(uint8_t *req_buffer)
{
  uint8_t idx = 0;
  
  // Frame structure: START, RD, PAYLOAD(9), DLE, EOF, CHECKSUM(4)
  req_buffer[idx++] = 0xA5;  // START
  req_buffer[idx++] = 0x13;  // RD
  req_buffer[idx++] = 0x06;  // PAYLOAD[0] = 0x06 (simple read)
  for (int i = 0; i < 8; i++) {
    req_buffer[idx++] = 0x00;  // PAYLOAD[1..8] zeros
  }
  req_buffer[idx++] = 0x10;  // DLE
  req_buffer[idx++] = 0x1F;  // EOF
  
  // Calculate checksum over bytes 0-12 (START through EOF)
  uint16_t cs = bldm_sum16(req_buffer, 13);
  uint8_t hi = (uint8_t)(cs >> 8);
  uint8_t lo = (uint8_t)(cs & 0xFF);
  
  // Append checksum as 4 nibble bytes
  req_buffer[idx++] = (hi >> 4) & 0x0F;
  req_buffer[idx++] = hi & 0x0F;
  req_buffer[idx++] = (lo >> 4) & 0x0F;
  req_buffer[idx++] = lo & 0x0F;
}

static void BuildI2CTxBuf(void)
{
  /* Populate TX buffer via UART sensor reply using BLDM sensor format */
  uint8_t bldm_request[17];
  float concentration_float = 0.0f;
  float temperature_float = 0.0f;
  float humidity_float = 0.0f;
  float absorptivity_float = 0.0f;

  /* CRITICAL: Flush any pending UART receive data before starting new transaction */
  /* This ensures we don't read stale data from previous transactions */
  uint8_t dummy;
  for (int i = 0; i < 32; i++) {  // Drain up to 32 bytes
    if (HAL_UART_Receive(&huart1, &dummy, 1, 1) != HAL_OK) {
      break;  // No more data available
    }
  }

  // Build BLDM request frame
  bldm_build_request(bldm_request);
  
  // Send BLDM request
  if (HAL_UART_Transmit(&huart1, bldm_request, sizeof(bldm_request), 100) != HAL_OK) {
    return;  // UART transmit failed - keep existing buffer data
  }
  
  // Receive BLDM response - expect 23 bytes
  // Frame structure: START(1) DAT(1) LEN(1) DATA(16) DLE(1) EOF(1) CHECK(2)
  const uint16_t bldm_response_length = 23;
  if (HAL_UART_Receive(&huart1, uart_rx_buffer, bldm_response_length, 500) != HAL_OK) {
    return;  // UART receive failed or timeout - keep existing buffer data
  }
  
  // Validate frame header
  if (uart_rx_buffer[0] != 0xA5) {
    return;  // Invalid START byte
  }
  if (uart_rx_buffer[1] != 0x1A) {
    return;  // Invalid CMD byte (expected DAT=0x1A)
  }
  if (uart_rx_buffer[2] != 0x10) {
    return;  // Invalid DATA_LEN (expected 0x10 = 16 bytes)
  }
  if (uart_rx_buffer[19] != 0x10 || uart_rx_buffer[20] != 0x1F) {
    return;  // Invalid DLE/EOF
  }
  
  // Verify checksum (bytes 0-20, checksum is bytes 21-22)
  uint16_t rx_checksum = ((uint16_t)uart_rx_buffer[21] << 8) | uart_rx_buffer[22];
  uint16_t calc_checksum = bldm_sum16(uart_rx_buffer, 21);
  if (rx_checksum != calc_checksum) {
    return;  // Checksum mismatch
  }
  
  // Parse 4 floats (little-endian) from bytes 3-18 (16 bytes total)
  // Byte order: bytes 3-6 = concentration, 7-10 = temperature, 11-14 = humidity, 15-18 = absorptivity
  memcpy(&concentration_float, &uart_rx_buffer[3],  4);
  memcpy(&temperature_float,  &uart_rx_buffer[7],  4);
  memcpy(&humidity_float,     &uart_rx_buffer[11], 4);
  memcpy(&absorptivity_float, &uart_rx_buffer[15], 4);
  
  // Convert floats to integer values for I2C transmission
  // Concentration: float -> uint32_t (in ppm, multiply by 100 to preserve 2 decimal places)
  uint32_t concentration_value = (uint32_t)(concentration_float * 100.0f + 0.5f);
  
  // Temperature: float -> uint16_t (in degrees C, multiply by 10 to preserve 1 decimal place)
  // Temperature can be negative, so we'll use offset encoding: add 1000 to make it always positive
  int16_t temp_int = (int16_t)(temperature_float * 10.0f + 0.5f);
  uint16_t temperature_value = (uint16_t)(temp_int + 1000);  // Offset by 1000 to handle negative values
  
  // Humidity: float -> uint8_t (in %, round to nearest integer)
  uint8_t humidity_value = (uint8_t)(humidity_float + 0.5f);
  if (humidity_value > 100) humidity_value = 100;  // Clamp to 0-100%
  
  // Absorptivity: float -> uint16_t (multiply by 1000 to preserve 3 decimal places)
  uint16_t absorptivity_value = (uint16_t)(absorptivity_float * 1000.0f + 0.5f);
  
  // Validate absorptivity value - if > 50000, likely invalid (negative value or corruption)
  // Set to 0 to indicate invalid reading
  if (absorptivity_value > 50000) {
    absorptivity_value = 0;
  }

  // Build I2C TX buffer
  uint8_t o = 0u;
  // Write board identifier (2 bytes: major.minor, e.g., 0x86 0x40 = "86-40")
  i2cTxBuf[o++] = GetSensorBoardIdMajor();  // Value 1: Board ID Major
  i2cTxBuf[o++] = GetSensorBoardIdMinor();  // Value 2: Board ID Minor
  i2cTxBuf[o++] = (uint8_t)((concentration_value >> 24) & 0xFF);  // Value 3: Concentration (MSB)
  i2cTxBuf[o++] = (uint8_t)((concentration_value >> 16) & 0xFF);
  i2cTxBuf[o++] = (uint8_t)((concentration_value >>  8) & 0xFF);
  i2cTxBuf[o++] = (uint8_t)((concentration_value      ) & 0xFF);  // (LSB)
  i2cTxBuf[o++] = (uint8_t)((temperature_value >> 8) & 0xFF);  // Value 4: Temperature (MSB)
  i2cTxBuf[o++] = (uint8_t)((temperature_value     ) & 0xFF);  // (LSB)
  i2cTxBuf[o++] = humidity_value;  // Value 5: Humidity
  i2cTxBuf[o++] = (uint8_t)((absorptivity_value >>  8) & 0xFF);  // Value 6: Absorptivity (MSB)
  i2cTxBuf[o++] = (uint8_t)((absorptivity_value      ) & 0xFF);  // (LSB)

  uint16_t adc = read_adc12_once();
  i2cTxBuf[o++] = (uint8_t)((adc >> 8) & 0xFF);  // Value 7: ADC Value (MSB)
  i2cTxBuf[o++] = (uint8_t)( adc       & 0xFF);  // (LSB)

  // Add calibration status bytes (7th and 8th bytes from Command 3 or 6 response)
  // CRITICAL: Always read from static variables to ensure consistency
  // The calibration sequences update these variables, and BuildI2CTxBuf preserves them
  // During suspension, BuildI2CTxBuf is not called (polling_suspended prevents it)
  // After suspension ends, the bytes are reset to invalid marker before BuildI2CTxBuf runs again
  i2cTxBuf[o++] = calibration_status_byte7;  // Byte 13: Calibration status byte 7
  i2cTxBuf[o++] = calibration_status_byte8;  // Byte 14: Calibration status byte 8

  // Calculate CalcConc from Absorptivity using quadratic equation (signed 32-bit, two's complement)
  // Equation: CalcConc = a * absorptivity^2 + b * absorptivity + c
  // coeff_c is coeff_c_rw (default at power-up, adjusted by Zero CalcConc in RAM)
  int32_t calc_conc = calculate_calc_conc((float)absorptivity_value);
  last_calc_conc_avg = calc_conc;  // For Zero CalcConc command: subtract this from coeff_c

  // Add CalcConc as signed 32-bit big-endian (two's complement)
  i2cTxBuf[o++] = (uint8_t)((calc_conc >> 24) & 0xFF);  // Byte 15: CalcConc MSB
  i2cTxBuf[o++] = (uint8_t)((calc_conc >> 16) & 0xFF);  // Byte 16: CalcConc byte 2
  i2cTxBuf[o++] = (uint8_t)((calc_conc >>  8) & 0xFF);  // Byte 17: CalcConc byte 3
  i2cTxBuf[o++] = (uint8_t)( calc_conc        & 0xFF);   // Byte 18: CalcConc LSB

  i2c_data_ready = 1u;
}

// Calculate CalcConc from Absorptivity using quadratic equation
// Equation: CalcConc = a * absorptivity^2 + b * absorptivity + c
// Note: No offset subtraction - absorptivity value used directly (unlike NDIR_LPHC_QuadEq)
// coeff_c is coeff_c_rw (default at power-up, adjusted by Zero CalcConc in RAM)
// Result is signed 32-bit (two's complement); negative values allowed
static int32_t calculate_calc_conc(float absorptivity)
{
  // Coefficients: coeff_a, coeff_b from sensor_types.h; coeff_c is coeff_c_rw (RAM, adjusted by Zero CalcConc)
  float calc_conc_float = sensor_quadratic_coeffs.coeff_a * absorptivity * absorptivity
                        + sensor_quadratic_coeffs.coeff_b * absorptivity
                        + coeff_c_rw;
  
  // No clamp: allow negative values (signed 32-bit). Clamp to int32_t range to avoid undefined conversion
  if (calc_conc_float > (float)INT32_MAX) {
    calc_conc_float = (float)INT32_MAX;
  } else if (calc_conc_float < (float)INT32_MIN) {
    calc_conc_float = (float)INT32_MIN;
  }
  
  // Convert to int32_t (round to nearest integer)
  return (int32_t)roundf(calc_conc_float);
}

// CRC-16 calculation with polynomial x^16 + x^15 + x^2 + 1 = 0x8005
// Matches Python implementation provided
static uint16_t crc16_custom(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0x0000;  // Initial value
  uint16_t poly = 0x8005;  // Polynomial
  
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = ((crc << 1) ^ poly) & 0xFFFF;
      } else {
        crc = (crc << 1) & 0xFFFF;
      }
    }
  }
  
  return crc;
}


// Send span sensor command over UART
// Format: FB 68 0C 00 00 53 00 01 xx xx xx xx CRC_H CRC_L FC
// where xx xx xx xx are the 4 span value bytes from ESP32
static void send_span_sensor_command(const uint8_t *span_bytes)
{
  // Build the UART command frame
  // Header: FB 68 0C (0x0C = 12 bytes of data following)
  // Fixed bytes: 00 00 53 00 01
  // Span value: span_bytes[0-3] (4 bytes from ESP32)
  // CRC: 2 bytes (calculated over FB 68 0C 00 00 53 00 01 + span_bytes)
  // Footer: FC
  
  uint8_t uart_cmd[16];  // Total frame size: 3 header + 12 data + 1 footer = 16 bytes
  uint8_t idx = 0;
  
  // Header
  uart_cmd[idx++] = 0xFB;
  uart_cmd[idx++] = 0x68;
  uart_cmd[idx++] = 0x0C;  // Length: 12 bytes of data
  
  // Fixed bytes
  uart_cmd[idx++] = 0x00;
  uart_cmd[idx++] = 0x00;
  uart_cmd[idx++] = 0x53;
  uart_cmd[idx++] = 0x00;
  uart_cmd[idx++] = 0x01;
  
  // Span value bytes (4 bytes from ESP32)
  uart_cmd[idx++] = span_bytes[0];
  uart_cmd[idx++] = span_bytes[1];
  uart_cmd[idx++] = span_bytes[2];
  uart_cmd[idx++] = span_bytes[3];
  
  // Calculate CRC over: FB 68 0C 00 00 53 00 01 + span_bytes (total 12 bytes)
  // CRC calculation range: from FB to end of span_bytes (12 bytes)
  uint16_t crc = crc16_custom(uart_cmd, 12);  // Calculate CRC over first 12 bytes (FB to span_bytes)
  
  // Append CRC (big-endian: high byte first, then low byte)
  uart_cmd[idx++] = (uint8_t)((crc >> 8) & 0xFF);  // CRC high byte
  uart_cmd[idx++] = (uint8_t)(crc & 0xFF);          // CRC low byte
  
  // Footer
  uart_cmd[idx++] = 0xFC;
  
  // Send complete frame over UART
  // Clear any pending UART data first
  uint8_t dummy;
  for (int i = 0; i < 32; i++) {
    if (HAL_UART_Receive(&huart1, &dummy, 1, 1) != HAL_OK) {
      break;
    }
  }
  
  // Transmit the span command
  HAL_UART_Transmit(&huart1, uart_cmd, sizeof(uart_cmd), 100);
}

// Receive calibration response from sensor after sending Zero or Span command
// Returns 1 if valid response received, 0 otherwise
static uint8_t receive_calibration_response(void)
{
  // Clear previous response
  calibration_response_length = 0u;
  calibration_response_ready = 0u;
  memset(calibration_response_buffer, 0, sizeof(calibration_response_buffer));
  
  // Wait for response frame - sensor typically responds with a frame starting with 0xFB 0x68
  // Try to receive up to 32 bytes with a reasonable timeout
  // First, try to receive the header (FB 68) to determine frame length
  uint8_t header[3];
  if (HAL_UART_Receive(&huart1, header, 3, 500) != HAL_OK) {
    return 0u;  // Timeout or error receiving header
  }
  
  // Validate header
  if (header[0] != 0xFB || header[1] != 0x68) {
    return 0u;  // Invalid header
  }
  
  // Byte 2 contains the length field - this indicates how many bytes follow (including footer)
  // Example: FB 68 08 00 A2 60 00 01 33 07 FC
  //          Length field = 08 means 8 bytes follow (7 data bytes + 1 footer byte)
  uint8_t bytes_following = header[2];
  uint8_t total_frame_length = 3 + bytes_following;  // Header(3) + bytes following (data + footer)
  
  // Sanity check: frame should be reasonable size
  if (total_frame_length > sizeof(calibration_response_buffer)) {
    return 0u;  // Frame too large
  }
  
  // Store header in response buffer
  calibration_response_buffer[0] = header[0];
  calibration_response_buffer[1] = header[1];
  calibration_response_buffer[2] = header[2];
  
  // Receive remaining bytes (data + footer)
  // The length field indicates exactly how many bytes follow (including the footer)
  // So we receive exactly 'bytes_following' bytes
  if (HAL_UART_Receive(&huart1, &calibration_response_buffer[3], bytes_following, 500) != HAL_OK) {
    return 0u;  // Timeout or error receiving data
  }
  
  // Validate footer (last byte should be 0xFC)
  // Total frame = 3 (header) + bytes_following (data + footer)
  // Last byte is at index: 3 + bytes_following - 1 = total_frame_length - 1
  if (calibration_response_buffer[total_frame_length - 1] != 0xFC) {
    return 0u;  // Invalid footer
  }
  
  // Valid response received
  calibration_response_length = total_frame_length;
  calibration_response_ready = 1u;
  return 1u;
}

// Send zero sensor calibration sequence: 6 commands with delays
// 1. Send command 1, receive response
// 2. Delay 13 seconds
// 3. Send command 2, receive response
// 4. Delay 5 seconds
// 5. Send command 3, receive response
// 6. Delay 10 seconds
// 7. Send command 4, receive response
// 8. Delay 13 seconds
// 9. Send command 5, receive response
// 10. Delay 5 seconds
// 11. Send command 6, receive response
static void send_zero_sensor_sequence(void)
{
  uint8_t response_ok;  // Variable to store receive_calibration_response() return value
  // Step 1: Send first zero calibration command
  HAL_UART_Transmit(&huart1, (uint8_t*)zero_sensor_uart_cmd_1, sizeof(zero_sensor_uart_cmd_1), 100);
  receive_calibration_response();  // Receive and store response
  
  // Step 2: Delay 13 seconds (13000ms) - increased from 3 seconds
  HAL_Delay(13000);
  
  // Step 3: Send second zero calibration command
  HAL_UART_Transmit(&huart1, (uint8_t*)zero_sensor_uart_cmd_2, sizeof(zero_sensor_uart_cmd_2), 100);
  receive_calibration_response();  // Receive and store response
  
  // Step 4: Delay 5 seconds (5000ms) - decreased from 10 seconds
  HAL_Delay(5000);
  
  // Step 5: Send third zero calibration command
  HAL_UART_Transmit(&huart1, (uint8_t*)zero_sensor_uart_cmd_3, sizeof(zero_sensor_uart_cmd_3), 100);
  // Receive response and extract calibration status bytes
  // receive_calibration_response() validates the frame structure and sets calibration_response_length
  // For a frame with length=0x08: total = 12 bytes, so we can safely access indices 6 and 7
  // If receive_calibration_response() returns 1, we have a valid frame with at least 8 bytes
  response_ok = receive_calibration_response();
  // Extract 7th and 8th bytes (indices 6 and 7) from response buffer ONLY if we got a valid response
  // Frame structure: FB 68 [length] [data...] FC
  // For length=0x08: total frame = 12 bytes (3 header + 8 data + 1 footer)
  // Bytes at indices 6 and 7 are the 7th and 8th bytes of the response frame
  // Example: FB 68 08 00 A2 60 00 01 33 07 FC
  //          Index: 0  1  2  3  4  5  6  7  8  9 10
  //          Byte 7 = index 6 = 0x00, Byte 8 = index 7 = 0x01
  if (response_ok != 0u) {
    // We have a valid response - check length to ensure safe array access
    if (calibration_response_length >= 8u) {
      calibration_status_byte7 = calibration_response_buffer[6];
      calibration_status_byte8 = calibration_response_buffer[7];
      calibration_status_valid = 1u;
      // Force update I2C buffer immediately with new calibration status
      // This ensures the status bytes are available for the next I2C read
      // Calibration status bytes are at indices 13-14 (after 2-byte board ID + 12 sensor data bytes)
      i2cTxBuf[13] = calibration_status_byte7;
      i2cTxBuf[14] = calibration_status_byte8;
    }
  }
  
  // Step 6: Delay 10 seconds (10000ms)
  HAL_Delay(10000);
  
  // Step 7: Send fourth zero calibration command
  HAL_UART_Transmit(&huart1, (uint8_t*)zero_sensor_uart_cmd_4, sizeof(zero_sensor_uart_cmd_4), 100);
  receive_calibration_response();  // Receive and store response
  
  // Step 8: Delay 13 seconds (13000ms) - increased from 3 seconds
  HAL_Delay(13000);
  
  // Step 9: Send fifth zero calibration command
  HAL_UART_Transmit(&huart1, (uint8_t*)zero_sensor_uart_cmd_5, sizeof(zero_sensor_uart_cmd_5), 100);
  receive_calibration_response();  // Receive and store response
  
  // Step 10: Delay 5 seconds (5000ms) - decreased from 10 seconds
  HAL_Delay(5000);
  
  // Step 11: Send sixth zero calibration command
  HAL_UART_Transmit(&huart1, (uint8_t*)zero_sensor_uart_cmd_6, sizeof(zero_sensor_uart_cmd_6), 100);
  // Receive response and extract calibration status bytes
  // receive_calibration_response() validates the frame structure and sets calibration_response_length
  // For a frame with length=0x08: total = 12 bytes, so we can safely access indices 6 and 7
  // If receive_calibration_response() returns 1, we have a valid frame with at least 8 bytes
  response_ok = receive_calibration_response();
  // Extract 7th and 8th bytes (indices 6 and 7) from response buffer ONLY if we got a valid response
  // Frame structure: FB 68 [length] [data...] FC
  // For length=0x08: total frame = 12 bytes (3 header + 8 data + 1 footer)
  // Bytes at indices 6 and 7 are the 7th and 8th bytes of the response frame
  // Example: FB 68 08 00 4C 60 00 00 6B 2F FC
  //          Index: 0  1  2  3  4  5  6  7  8  9 10
  //          Byte 7 = index 6 = 0x00, Byte 8 = index 7 = 0x00
  if (response_ok != 0u) {
    // We have a valid response - check length to ensure safe array access
    if (calibration_response_length >= 8u) {
      calibration_status_byte7 = calibration_response_buffer[6];
      calibration_status_byte8 = calibration_response_buffer[7];
      calibration_status_valid = 1u;
      // Force update I2C buffer immediately with new calibration status
      // This ensures the status bytes are available for the next I2C read
      // Calibration status bytes are at indices 13-14 (after 2-byte board ID + 12 sensor data bytes)
      i2cTxBuf[13] = calibration_status_byte7;
      i2cTxBuf[14] = calibration_status_byte8;
    }
  }
}

/* USER CODE END 0: helpers */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ticks_without_i2c_comm = 0uL;
uint32_t uart_reply_errors_detected = 0uL;

uint32_t recorded_errors = 0uL;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Load coeff_c from Flash (or use default if Flash is empty/invalid) */
  if (!read_coeff_c_from_flash(&coeff_c_rw)) {
    // Flash is empty or invalid - use default value (first time only)
    coeff_c_rw = sensor_quadratic_coeffs.coeff_c;
  }

  /* Initialize I2C TX buffer with Board ID (2 bytes) */
  i2cTxBuf[0] = GetSensorBoardIdMajor();
  i2cTxBuf[1] = GetSensorBoardIdMinor();
  for (uint8_t i = 2; i < 18; i++) {
    i2cTxBuf[i] = 0;  // Clear sensor data bytes (up to CalcConc)
  }
  // Initialize calibration status bytes to invalid marker (will display as "xxxx")
  i2cTxBuf[13] = CAL_STATUS_INVALID_MARKER;
  i2cTxBuf[14] = CAL_STATUS_INVALID_MARKER;
  // Initialize CalcConc to zero (bytes 15-18 after board ID[2] + data[13] + cal status[2])
  i2cTxBuf[15] = 0;
  i2cTxBuf[16] = 0;
  i2cTxBuf[17] = 0;
  i2cTxBuf[18] = 0;

  /* Start listening as I2C slave at address 0x60 */
  HAL_I2C_EnableListen_IT(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  volatile uint32_t just_ticks = 0uL;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      just_ticks++;
      
      /* Handle zero sensor command if pending (deferred from I2C callback to avoid blocking) */
      if (zero_sensor_pending && !i2c_busy) {
        zero_sensor_pending = 0u;  // Clear flag first to prevent re-entry
        
        /* Clear any pending UART data before sending to avoid interfering with BuildI2CTxBuf */
        uint8_t dummy;
        for (int i = 0; i < 32; i++) {  // Drain up to 32 bytes (larger than typical frame)
          if (HAL_UART_Receive(&huart1, &dummy, 1, 1) != HAL_OK) {
            break;  // No more data available
          }
        }
        
        /* Send zero sensor calibration sequence (3 commands with delays and responses) */
        send_zero_sensor_sequence();
        
        /* Suspend data polling for 20000ms (20 seconds) after zero sensor command sequence */
        polling_suspend_until = HAL_GetTick() + 20000uL;
      }
      
      /* Handle span sensor command if pending (deferred from I2C callback to avoid blocking) */
      if (span_sensor_pending && !i2c_busy) {
        span_sensor_pending = 0u;  // Clear flag first to prevent re-entry
        
        /* Send the span sensor command with received span value bytes */
        send_span_sensor_command(span_value_bytes);
        
        /* Receive and store the sensor's response */
        receive_calibration_response();
        
        /* Suspend data polling for 60000ms after span sensor command */
        polling_suspend_until = HAL_GetTick() + 60000uL;
      }

      /* Handle Zero CalcConc command: adjust coeff_c to zero CalcConc and save to Flash */
      if (zero_calc_conc_pending && !i2c_busy) {
        zero_calc_conc_pending = 0u;
        
        // Calculate adjustment needed to zero CalcConc
        // To zero CalcConc: new_coeff_c = old_coeff_c - CalcConc
        // This works for both positive and negative CalcConc values
        float adjustment = -(float)last_calc_conc_avg;
        coeff_c_rw += adjustment;  // Add adjustment (subtract if CalcConc is positive, add if negative)
        
        // Save updated coeff_c to Flash (non-volatile storage)
        write_coeff_c_to_flash(coeff_c_rw);
      }
      
      /* Update I2C TX buffer periodically (e.g., every 500 ms) to keep buffer fresh */
      /* But don't run if I2C is busy to avoid interference */
      /* Also suspend polling for 20000ms (20s) after zero sensor or 60000ms (60s) after span sensor commands */
      static uint32_t last_build = 0uL;
      uint32_t now = HAL_GetTick();
      
      /* Check if polling is suspended (after zero/span command) */
      /* Use difference-based comparison to handle tick wrap-around correctly */
      uint8_t polling_suspended = 0u;
      if (polling_suspend_until != 0uL) {
        /* Calculate time remaining until suspension expires */
        /* If result is > 0x7FFFFFFF, it means wrap-around occurred (suspension expired) */
        uint32_t time_remaining = polling_suspend_until - now;
        /* Check if still suspended: time_remaining must be > 0, <= 60000 (max for span sensor), and not wrapped */
        if (time_remaining > 0uL && time_remaining <= 60000uL && time_remaining < 0x80000000uL) {
          polling_suspended = 1u;  // Still suspended (time remaining is valid and <= max suspension time)
        } else {
          // Suspension period expired - reset to normal operation (match LPHC: reset cal bytes and CalcConc)
          calibration_status_byte7 = CAL_STATUS_INVALID_MARKER;
          calibration_status_byte8 = CAL_STATUS_INVALID_MARKER;
          calibration_status_valid = 0u;
          i2cTxBuf[13] = CAL_STATUS_INVALID_MARKER;
          i2cTxBuf[14] = CAL_STATUS_INVALID_MARKER;
          i2cTxBuf[15] = 0;
          i2cTxBuf[16] = 0;
          i2cTxBuf[17] = 0;
          i2cTxBuf[18] = 0;
          polling_suspend_until = 0uL;
        }
      }
      
      if ((now - last_build) >= 500u && !i2c_busy && !polling_suspended) {
          BuildI2CTxBuf();
          last_build = now;
      }
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
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  
  // Perform ADC calibration for improved accuracy
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = (THIS_SENSOR_BOARD_I2C_ADDRESS << 1);
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;  // Enable clock stretching - allows slave to pause clock while preparing data
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	  recorded_errors++; // Place a Break-Point here.
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
