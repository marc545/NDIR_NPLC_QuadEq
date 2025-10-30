/*
 * sensor_types.h
 *
 *  Created on: Oct 30, 2025
 *  Authors: taduri.fwdev@outlook.com
 */

#ifndef SENSOR_TYPES_H_
#define SENSOR_TYPES_H_

enum gas_sensor_types_and_ids_e {
	GAS_SENSOR_TYPE__AAA_ID = 0x11,
	GAS_SENSOR_TYPE__BBB_ID = 0x12,
	GAS_SENSOR_TYPE__CCC_ID = 0x13,
	GAS_SENSOR_TYPE__DDD_OD = 0x14
};

enum gas_sensor_types_and_ids_e {
	GAS_SENSOR_TYPE__AAA_I2C_ADDR = 0x60,
	GAS_SENSOR_TYPE__BBB_I2C_ADDR = 0x61,
	GAS_SENSOR_TYPE__CCC_I2C_ADDR = 0x62,
	GAS_SENSOR_TYPE__DDD_I2C_ADDR = 0x63,
};

#define THIS_SENSOR_BOARD_ID           GAS_SENSOR_TYPE__AAA_ID
#define THIS_SENSOR_BOARD_I2C_ADDRESS  GAS_SENSOR_TYPE__AAA_I2C_ADDR

extern uint8_t sensor_type_id;
extern uint8_t sensor_i2c_addr;

#endif /* SENSOR_TYPES_H_ */
