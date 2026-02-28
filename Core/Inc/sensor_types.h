/*
 * sensor_types.h
 *
 *  Created on: Oct 30, 2025
 *  Authors: taduri.fwdev@outlook.com
 */
#include <stdint.h>

#ifndef SENSOR_TYPES_H_
#define SENSOR_TYPES_H_

enum gas_sensor_types_and_ids_e {
	GAS_SENSOR_TYPE__AAA_ID = 0x11,
	GAS_SENSOR_TYPE__BBB_ID = 0x12,
	GAS_SENSOR_TYPE__CCC_ID = 0x13,
	GAS_SENSOR_TYPE__DDD_OD = 0x14
};

enum gas_sensor_i2c_addr_range_e {
	GAS_SENSOR_TYPE__AAA_I2C_ADDR = 0x60,
	GAS_SENSOR_TYPE__BBB_I2C_ADDR = 0x61,
	GAS_SENSOR_TYPE__CCC_I2C_ADDR = 0x62,
	GAS_SENSOR_TYPE__DDD_I2C_ADDR = 0x63,
};

#define THIS_SENSOR_BOARD_ID           GAS_SENSOR_TYPE__AAA_ID
#define THIS_SENSOR_BOARD_I2C_ADDRESS  GAS_SENSOR_TYPE__AAA_I2C_ADDR

// Board identifier (2 bytes: major.minor format, e.g., 0x86 0x40 = "86-40")
// TODO: Set these values for each board (e.g., 86-40, 86-41, 86-42, 86-43)
#define THIS_SENSOR_BOARD_ID_MAJOR     0x86  // First part of board ID (e.g., "86" in "86-40")
#define THIS_SENSOR_BOARD_ID_MINOR     0x36  // Second part of board ID (e.g., "40" in "86-40")

// Quadratic equation coefficients for Absorptivity to CalcConc conversion
// These are unique per sensor board and should be calibrated for each sensor
// Equation: CalcConc = a * absorptivity^2 + b * absorptivity + c
// Note: No offset subtraction (absorptivity value used directly)
typedef struct {
  float coeff_a;     // Quadratic coefficient
  float coeff_b;     // Linear coefficient
  float coeff_c;     // Constant coefficient
} quadratic_coeffs_t;

// Sensor-specific quadratic coefficients
// TODO: Calibrate these values for each individual sensor board
static const quadratic_coeffs_t sensor_quadratic_coeffs = {
  .coeff_a = 0.00386f,
  .coeff_b = 45.35f,
  .coeff_c = 1.0f
};

uint8_t GetSensorBoardId(void);
uint8_t GetSensorBoardI2Caddress(void);
uint8_t GetSensorBoardIdMajor(void);  // Returns major part of board ID (e.g., 0x86 for "86-40")
uint8_t GetSensorBoardIdMinor(void);  // Returns minor part of board ID (e.g., 0x40 for "86-40")

#endif /* SENSOR_TYPES_H_ */
