/*
 * vl6180.c
 *
 *  Created on: May 14, 2025
 *      Author: Stefano, Eugen
 */

#include "vl6180.h"
#include "main.h"
#include <stdio.h>
#include "i2c.h"


/* VL6180X Register Access Functions */

uint8_t VL6180X_WriteReg(uint8_t sensor_addr, uint16_t reg, uint8_t value) {
	uint8_t data[3] = { (reg >> 8) & 0xFF, reg & 0xFF, value };
	return HAL_I2C_Master_Transmit(&hi2c1, sensor_addr << 1, data, 3, 10);
}

uint8_t VL6180X_ReadReg(uint8_t sensor_addr, uint16_t reg) {
	uint8_t tx_data[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
	uint8_t rx_data = 0;
	HAL_I2C_Master_Transmit(&hi2c1, sensor_addr << 1, tx_data, 2, 30);
	HAL_I2C_Master_Receive(&hi2c1, sensor_addr << 1, &rx_data, 1, 30);
	return rx_data;
}


void VL6180X_Setup(VL6180X_Sensor *sensors) {
	// Sensor 0 - left
	sensors[0].addr = VL6180X_1_ADDR;
	sensors[0].xshut_pin  = VL6180X_1_SHUT_Pin;
	sensors[0].xshut_port = VL6180X_1_SHUT_GPIO_Port;
	sensors[0].working = false;
	sensors[0].m = VL6180X_1_M;
	sensors[0].q = VL6180X_1_Q;

	// Sensor 1 - front
	sensors[1].addr = VL6180X_2_ADDR;
	sensors[1].xshut_pin  = VL6180X_2_SHUT_Pin;
	sensors[1].xshut_port = VL6180X_2_SHUT_GPIO_Port;
	sensors[1].working = false;
	sensors[1].m = VL6180X_2_M;
	sensors[1].q = VL6180X_2_Q;
}

uint8_t VL6180X_Init(uint8_t sensor_addr) {
	if (VL6180X_ReadReg(sensor_addr, VL6180X_IDENTIFICATION_MODEL_ID) != VL6180X_ID) {
		return 1;
	}

	VL6180X_WriteReg(sensor_addr, 0x0207, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x0208, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x0096, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x0097, 0xFD);
	VL6180X_WriteReg(sensor_addr, 0x00E3, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00E4, 0x04);
	VL6180X_WriteReg(sensor_addr, 0x00E5, 0x02);
	VL6180X_WriteReg(sensor_addr, 0x00E6, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x00E7, 0x03);
	VL6180X_WriteReg(sensor_addr, 0x00F5, 0x02);
	VL6180X_WriteReg(sensor_addr, 0x00D9, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x00DB, 0xCE);
	VL6180X_WriteReg(sensor_addr, 0x00DC, 0x03);
	VL6180X_WriteReg(sensor_addr, 0x00DD, 0xF8);
	VL6180X_WriteReg(sensor_addr, 0x009F, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00A3, 0x3C);
	VL6180X_WriteReg(sensor_addr, 0x00B7, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00BB, 0x3C);
	VL6180X_WriteReg(sensor_addr, 0x00B2, 0x09);
	VL6180X_WriteReg(sensor_addr, 0x00CA, 0x09);
	VL6180X_WriteReg(sensor_addr, 0x0198, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x01B0, 0x17);
	VL6180X_WriteReg(sensor_addr, 0x01AD, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00FF, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x0100, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x0199, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x01A6, 0x1B);
	VL6180X_WriteReg(sensor_addr, 0x01AC, 0x3E);
	VL6180X_WriteReg(sensor_addr, 0x01A7, 0x1F);
	VL6180X_WriteReg(sensor_addr, 0x0030, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x0011, 0x10);
	VL6180X_WriteReg(sensor_addr, 0x012A, 0x02);
	VL6180X_WriteReg(sensor_addr, 0x003A, 0x03);
	VL6180X_WriteReg(sensor_addr, 0x0031, 0xFF);
	VL6180X_WriteReg(sensor_addr, 0x0014, 0x24);

	return 0;
}

void VL6180X_i2c_Init(VL6180X_Sensor *sensors) {

	/* IMPORTANT!!!
	 * (for old sensors VL6180X_2 HAS THE SCRATCH)
	 * swapping sensor may lead missreading
	 * due to calibration of each VL6180X
	 * with separate linear regression
	 */

	// shut down all not working sensors
	for (uint8_t i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		if (!sensors[i].working) {
			HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);
		}
	}
	//printf("Scanning, nothing should be found\n");
	//i2cScan();

	/*
	//test
	for (uint8_t i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		printf("testing %d -   ", i);
		HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_SET);
		HAL_Delay(5);
		i2cScan();
		HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);
	}
	printf("Scanning, should be nothing found \n");
	i2cScan();
	*/

	// for all not working sensor
	// turn them on one by one initialize them
	for (uint8_t i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		if (!sensors[i].working) {
			HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_SET);
			HAL_Delay(5);
			VL6180X_WriteReg(VL6180X_DEFAULT_ADDR, VL6180X_REG_I2C_SLAVE_DEV_ADDR, sensors[i].addr);
			HAL_Delay(5);
			if (VL6180X_Init(sensors[i].addr) == 0) {
				sensors[i].working = true;
			}
		}
	}
	//i2cScan();
}


void VL6180X_SendMsgToRead(VL6180X_Sensor *sensors) {

	// Send the input signal to read

	for (int i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		if (sensors[i].working) {
			VL6180X_WriteReg(sensors[i].addr, VL6180X_SYSRANGE_START, 0x01);
		}
	}
}

void VL6180X_ReadDistances(VL6180X_Sensor *sensors, uint8_t *counter) {

	// Minimum 60 ms after calling VL6180X_SendMsgToRead
	// Read values from sensors

	uint16_t readValue = 0;
	uint8_t flag = 0;

	for (int i = 0; i < NUMBER_OF_VL6180X_ID; i++) {  // Loop over all sensors
		if (sensors[i].working) {  // Check if the sensor is working

			// Check if the measurement is complete (based on the result range status)
			if ((VL6180X_ReadReg(sensors[i].addr, VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0) {

				/* Error reading ith VL6180X */
				sensors[i].working = false;
				sensors[i].distance = 0;  // Set distance to 0 as error state

				// Shutdown the sensor (disable it)
				HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);

			} else {
				// Sensor read correctly, process the reading
				readValue = VL6180X_ReadReg(sensors[i].addr, VL6180X_RESULT_RANGE_VAL);

				// Reset the sensor's fresh out of reset register
				//VL6180X_WriteReg(sensors[i].addr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);

				// Apply linear transformation (using m and q from the sensor's configuration)
				readValue = sensors[i].m * readValue + sensors[i].q;

				// Clamp the read value to a valid range (0-255)
				if (readValue > 255) {
					sensors[i].distance = 255;
				} else if (readValue < 5) {
					sensors[i].distance = 0;
				} else {
					sensors[i].distance = readValue;
				}
			}
		} else {
			// If the sensor is not working, increment the counter
			flag = 1;

		}
	}
	if (flag) (*counter)++;
}
















uint8_t VL6180X_ReadDistancePolling(uint8_t sensor_addr, uint8_t *distance) {
	uint32_t start_time = HAL_GetTick();
	const uint32_t timeout_ms = 80;

	VL6180X_WriteReg(sensor_addr, VL6180X_SYSRANGE_START, 0x01);

	while ((VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0) {
		if ((HAL_GetTick() - start_time) > timeout_ms) {
			return 1;
		}
	}

	*distance = VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_VAL);
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return 0;
}

void VL6180X_ReadAllDistancesPolling(VL6180X_Sensor *sensors, uint8_t *counter) {
	for (int i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		if (sensors[i].working) {
			if (VL6180X_ReadDistancePolling(sensors[i].addr, &sensors[i].distance) == 0) {
				if ((int)(sensors[i].m * sensors[i].distance + sensors[i].q) <= 255){
					sensors[i].distance = (int)(sensors[i].m * sensors[i].distance + sensors[i].q);
				} else {
					sensors[i].distance = 255;
				}
			} else {
				/* Error reading ith VL6180X */
				sensors[i].working = false;

				/* Shutdown that VL6180X */
				HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);
			}
		} else {
			/* ith VL6180X not working */
			(*counter)++;
		}
	}
}

void VL6180X_PrintAllDistances(VL6180X_Sensor *sensors){
	for (int i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		printf("Sensor %d  ",i+1);
		if (sensors[i].working) {
			if (VL6180X_ReadDistancePolling(sensors[i].addr, &sensors[i].distance) == 0) {
				printf("measured %d, corrected %d ",
						sensors[i].distance,
						(int)(sensors[i].m * sensors[i].distance + sensors[i].q));
			}
		} else {
			printf("--- ");
		}
		printf("     |     ");
	}
	printf("\r\n");
}

void i2cScan() {
	uint8_t c = 0;
	for (uint8_t addr = 0x03; addr < 0x77; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 10) == HAL_OK) {
			printf("Found device at 0x%02X\n", addr);
			c++;
		}
	}
	if (c == 0) {
		printf("No device found \n");
	}
}

















HAL_StatusTypeDef VL6180X_TryReadDistance(VL6180X_Sensor *sensor, uint8_t *distance_out) {
    uint8_t range_status;
    uint8_t raw_distance;
    HAL_StatusTypeDef status;

    // Check measurement ready (range status)
    status = HAL_I2C_Mem_Read(&hi2c1, sensor->addr, VL6180X_RESULT_RANGE_STATUS,
                              I2C_MEMADD_SIZE_8BIT, &range_status, 1, 10);
    if (status != HAL_OK) {
        return status;  // I2C error: sensor missing or bus issue
    }

    // Check if measurement is ready (LSB bit should be 1)
    if ((range_status & 0x01) == 0) {
        return HAL_ERROR;  // Measurement not ready or bad data
    }

    // Read distance value
    status = HAL_I2C_Mem_Read(&hi2c1, sensor->addr, VL6180X_RESULT_RANGE_VAL,
                              I2C_MEMADD_SIZE_8BIT, &raw_distance, 1, 10);
    if (status != HAL_OK) {
        return status;
    }

    // Apply calibration and clamping
    int value = sensor->m * raw_distance + sensor->q;
    if (value > 255) {
        *distance_out = 255;
    } else if (value < 5) {
        *distance_out = 0;
    } else {
        *distance_out = value;
    }

    return HAL_OK;
}

void VL6180X_ReadAllDistances(VL6180X_Sensor *sensors, uint8_t *offline_counter) {
    HAL_StatusTypeDef status;
    uint8_t distance;
    uint8_t flag = 0;

    for (int i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
        if (sensors[i].working) {
            status = VL6180X_TryReadDistance(&sensors[i], &distance);

            if (status == HAL_OK) {
                sensors[i].distance = distance;
            } else {
                // If communication failed or bad data, mark sensor offline
                sensors[i].working = false;
                sensors[i].distance = 0;
                HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);
                flag = 1;
            }
        } else {
            flag = 1;  // already offline
        }
    }

    if (flag) (*offline_counter)++;
}
