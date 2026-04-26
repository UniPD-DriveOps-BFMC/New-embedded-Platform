/*
 * vl6180.h
 *
 *  Created on: May 14, 2025
 *      Author: Stefano, Eugen
 */

#ifndef INC_VL6180_H_
#define INC_VL6180_H_

#include "stm32f4xx_hal.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUMBER_OF_VL6180X_ID          		2
typedef struct {
	uint8_t addr;
	uint16_t xshut_pin;
	GPIO_TypeDef* xshut_port;
	bool working;
	uint8_t distance;
	float m;
	float q;
	//uint8_t errorCounter;
} VL6180X_Sensor;

/* VL6180X sensor ID */
#define VL6180X_ID                    		0xB4

/* Address */
//#define VL6180X_ADDR                  	  	0x52
#define VL6180X_DEFAULT_ADDR          		0x29
#define VL6180X_1_ADDR          			0x30
#define VL6180X_2_ADDR          			0x31

/* GPIO configuration for SHUT are in main.h
#define VL6180X_1_SHUT_PIN		      		GPIO_PIN_9
#define VL6180X_1_SHUT_PORT           		GPIOC
#define VL6180X_2_SHUT_PIN		      		GPIO_PIN_8
#define VL6180X_2_SHUT_PORT           		GPIOC
*/

/* Register addresses */
#define VL6180X_REG_I2C_SLAVE_DEV_ADDR      0x0212
#define VL6180X_IDENTIFICATION_MODEL_ID     0x0000
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET   0x0016
#define VL6180X_SYSRANGE_START              0x0018
#define VL6180X_RESULT_RANGE_STATUS         0x004D
#define VL6180X_RESULT_RANGE_VAL            0x0062

/* Linear correction parameters (y = mx + q) */
#define VL6180X_1_M  						0.97053299
#define VL6180X_1_Q  						19.55846396
#define VL6180X_2_M  						0.60646845// old tof 0.96133608
#define VL6180X_2_Q  						29.31900131// old tof 0.36225924

/* Counter for reinitialize VL6180X sensor(s) */
#define VL6180X_REINITIALIZE_COUNTER  		30*(NUMBER_OF_VL6180X_ID)

/* Public function prototypes */
uint8_t VL6180X_WriteReg(uint8_t sensor_addr, uint16_t reg, uint8_t value);
uint8_t VL6180X_ReadReg(uint8_t sensor_addr, uint16_t reg);
void VL6180X_Setup(VL6180X_Sensor *sensors);
uint8_t VL6180X_Init(uint8_t sensor_addr);
void VL6180X_i2c_Init(VL6180X_Sensor *sensors);
void VL6180X_SendMsgToRead(VL6180X_Sensor *sensors);
void VL6180X_ReadDistances(VL6180X_Sensor *sensors, uint8_t *counter);
// these functions use polling to read, use for debug
uint8_t VL6180X_ReadDistancePolling(uint8_t sensor_addr, uint8_t *distance);
void VL6180X_ReadAllDistancesPolling(VL6180X_Sensor *sensors, uint8_t *counter);
void VL6180X_PrintAllDistances(VL6180X_Sensor *sensors);
void i2cScan();


HAL_StatusTypeDef VL6180X_TryReadDistance(VL6180X_Sensor *sensor, uint8_t *distance_out);
void VL6180X_ReadAllDistances(VL6180X_Sensor *sensors, uint8_t *offline_counter);

#ifdef __cplusplus
}
#endif

#endif /* INC_VL6180_H_ */
