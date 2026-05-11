#ifndef INC_TOF_H_
#define INC_TOF_H_

#include "main.h"
#include <stdint.h>

#define TOF_SENSOR_COUNT 4

/*
 * Public API.
 *
 * Implementation is a C port of the Pololu VL6180X Arduino library
 * (https://github.com/pololu/vl6180x-arduino), adapted for STM32 HAL
 * and a multi-sensor XSHUT setup.
 *
 * Init policy: best-effort per sensor. A failure on one sensor does
 * NOT abort the others. TOF_Init() returns a bitmask of successfully
 * initialized sensors (bit i = sensor i). Expected value: 0x0F.
 */

uint8_t TOF_Init(void);
uint8_t TOF_ReadDistance(uint8_t sensor_index);
void    TOF_ReadAll(volatile uint8_t distances[TOF_SENSOR_COUNT]);

uint8_t TOF_GetInitStatus(void);
uint8_t TOF_GetAddressMask(void);
uint8_t TOF_GetLastRangeStatus(uint8_t sensor_index);
uint8_t TOF_IsReady(uint8_t sensor_index);

/* Per-sensor init diagnostic. 0 = success.
 * Encoding (non-zero values):
 *   0x01  0x29 not present after XSHUT raised  (sensor dead / not booting)
 *   0x02  model-ID mismatch at 0x29            (not a VL6180X / bus glitch)
 *   0x03  address-change write failed          (I2C error)
 *   0x04  0x29 still present after addr change (multiple sensors at 0x29)
 *   0x05  new address does not respond         (address change didn't stick)
 *   0x06  model-ID mismatch at new address
 *   0x07  init() (private registers) failed
 *   0x09  configureDefault() failed
 */
uint8_t TOF_GetInitFailStep(uint8_t sensor_index);

/* Set if a sensor was awake at 0x29 BEFORE any XSHUT was raised
 * (i.e. its SHDN circuit isn't actually shutting it down).
 * Informational - init still tries all sensors. */
uint8_t TOF_GetPreInitRogueFlag(void);

#endif
