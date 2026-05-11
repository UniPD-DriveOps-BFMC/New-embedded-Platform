/*
 * tof.c
 *
 * VL6180X driver for STM32 HAL, multi-sensor (4 sensors on one bus,
 * disambiguated via XSHUT and unique I2C addresses).
 *
 * Init policy: best-effort per sensor. A failure on one sensor does
 * NOT abort the others. TOF_Init() returns the bitmask of sensors
 * that initialized successfully.
 *
 * Implementation derived from the Pololu VL6180X Arduino library
 * (https://github.com/pololu/vl6180x-arduino), translated to C and
 * adapted for STM32 HAL. Init / configureDefault / read sequences
 * follow the same registers and values as Pololu's library, which in
 * turn follows ST application note AN4545.
 */

#include "tof.h"
#include "i2c.h"

/* ------------------------------------------------------------------ */
/* Addresses                                                           */
/* ------------------------------------------------------------------ */

#define VL6180X_DEFAULT_7BIT   0x29

#define TOF1_7BIT  0x2A
#define TOF2_7BIT  0x3A
#define TOF3_7BIT  0x4A
#define TOF4_7BIT  0x5A

#define ADDR8(a7) ((uint16_t)((a7) << 1))

/* ------------------------------------------------------------------ */
/* VL6180X registers                                                   */
/* ------------------------------------------------------------------ */

#define IDENTIFICATION__MODEL_ID              0x000
#define SYSTEM__INTERRUPT_CONFIG_GPIO         0x014
#define SYSTEM__INTERRUPT_CLEAR               0x015
#define SYSTEM__FRESH_OUT_OF_RESET            0x016

#define SYSRANGE__START                       0x018
#define SYSRANGE__INTERMEASUREMENT_PERIOD     0x01B
#define SYSRANGE__MAX_CONVERGENCE_TIME        0x01C
#define SYSRANGE__VHV_RECALIBRATE             0x02E
#define SYSRANGE__VHV_REPEAT_RATE             0x031

#define SYSALS__INTERMEASUREMENT_PERIOD       0x03E
#define SYSALS__ANALOGUE_GAIN                 0x03F
#define SYSALS__INTEGRATION_PERIOD            0x040

#define RESULT__RANGE_STATUS                  0x04D
#define RESULT__INTERRUPT_STATUS_GPIO         0x04F
#define RESULT__RANGE_VAL                     0x062

#define READOUT__AVERAGING_SAMPLE_PERIOD      0x10A
#define I2C_SLAVE__DEVICE_ADDRESS             0x212
#define INTERLEAVED_MODE__ENABLE              0x2A3

#define VL6180X_MODEL_ID_VALUE                0xB4

/* ------------------------------------------------------------------ */
/* Timing                                                              */
/* ------------------------------------------------------------------ */

#define TOF_BOOT_DELAY_MS                20    /* > t_BOOT (1.4 ms typ) */
#define TOF_FULL_RESET_DELAY_MS          100
#define TOF_ADDRESS_CHANGE_DELAY_MS      20
#define TOF_I2C_TIMEOUT_MS               50
#define TOF_RANGE_TIMEOUT_MS             120
#define TOF_BETWEEN_SENSOR_READ_DELAY_MS 5

/* ------------------------------------------------------------------ */
/* Per-sensor state                                                    */
/* ------------------------------------------------------------------ */

typedef struct
{
    GPIO_TypeDef *xshut_port;
    uint16_t      xshut_pin;
    uint8_t       addr_7bit;
    uint8_t       initialized;
    uint8_t       last_range_status;
    uint8_t       init_fail_step;
} TOF_Sensor_t;

static TOF_Sensor_t tof_sensors[TOF_SENSOR_COUNT] =
{
    { VL6180X_1_SHUT_GPIO_Port, VL6180X_1_SHUT_Pin, TOF1_7BIT, 0, 0, 0 },
    { VL6180X_2_SHUT_GPIO_Port, VL6180X_2_SHUT_Pin, TOF2_7BIT, 0, 0, 0 },
    { VL6180X_3_SHUT_GPIO_Port, VL6180X_3_SHUT_Pin, TOF3_7BIT, 0, 0, 0 },
    { VL6180X_4_SHUT_GPIO_Port, VL6180X_4_SHUT_Pin, TOF4_7BIT, 0, 0, 0 }
};

static uint8_t tof_init_status = 0;
static uint8_t tof_pre_init_rogue = 0;

/* ------------------------------------------------------------------ */
/* Public accessors                                                    */
/* ------------------------------------------------------------------ */

uint8_t TOF_GetInitStatus(void)        { return tof_init_status; }
uint8_t TOF_GetPreInitRogueFlag(void)  { return tof_pre_init_rogue; }

uint8_t TOF_IsReady(uint8_t i)
{
    return (i < TOF_SENSOR_COUNT) ? tof_sensors[i].initialized : 0;
}

uint8_t TOF_GetLastRangeStatus(uint8_t i)
{
    return (i < TOF_SENSOR_COUNT) ? tof_sensors[i].last_range_status : 0xFF;
}

uint8_t TOF_GetInitFailStep(uint8_t i)
{
    return (i < TOF_SENSOR_COUNT) ? tof_sensors[i].init_fail_step : 0xFF;
}

/* ------------------------------------------------------------------ */
/* I2C primitives + bus recovery                                        */
/* ------------------------------------------------------------------ */

static uint8_t TOF_AddressReady(uint16_t addr_8bit)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, addr_8bit, 2,
                                  TOF_I2C_TIMEOUT_MS) == HAL_OK);
}

uint8_t TOF_GetAddressMask(void)
{
    uint8_t m = 0;
    if (TOF_AddressReady(ADDR8(0x29))) m |= (1U << 0);
    if (TOF_AddressReady(ADDR8(0x2A))) m |= (1U << 1);
    if (TOF_AddressReady(ADDR8(0x3A))) m |= (1U << 2);
    if (TOF_AddressReady(ADDR8(0x4A))) m |= (1U << 3);
    if (TOF_AddressReady(ADDR8(0x5A))) m |= (1U << 4);
    return m;
}

static void TOF_I2C_Recover(void)
{
    if (hi2c1.State != HAL_I2C_STATE_READY ||
        hi2c1.ErrorCode != HAL_I2C_ERROR_NONE)
    {
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(2);
        HAL_I2C_Init(&hi2c1);
        __HAL_I2C_ENABLE(&hi2c1);
    }
}

static HAL_StatusTypeDef writeReg(uint8_t dev7, uint16_t reg, uint8_t value)
{
    uint8_t buf[3];
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    buf[2] = value;
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, ADDR8(dev7),
                                                   buf, 3, TOF_I2C_TIMEOUT_MS);
    if (st != HAL_OK) TOF_I2C_Recover();
    return st;
}

static HAL_StatusTypeDef writeReg16(uint8_t dev7, uint16_t reg, uint16_t value)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)(value & 0xFF);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, ADDR8(dev7),
                                                   buf, 4, TOF_I2C_TIMEOUT_MS);
    if (st != HAL_OK) TOF_I2C_Recover();
    return st;
}

static HAL_StatusTypeDef readReg(uint8_t dev7, uint16_t reg, uint8_t *value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    if (HAL_I2C_Master_Transmit(&hi2c1, ADDR8(dev7), buf, 2,
                                TOF_I2C_TIMEOUT_MS) != HAL_OK)
    {
        TOF_I2C_Recover();
        return HAL_ERROR;
    }
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&hi2c1, ADDR8(dev7),
                                                  value, 1, TOF_I2C_TIMEOUT_MS);
    if (st != HAL_OK) TOF_I2C_Recover();
    return st;
}

/* ------------------------------------------------------------------ */
/* GPIO helpers                                                        */
/* ------------------------------------------------------------------ */

static void TOF_DisableSensor(uint8_t i)
{
    HAL_GPIO_WritePin(tof_sensors[i].xshut_port,
                      tof_sensors[i].xshut_pin,
                      GPIO_PIN_RESET);
    tof_sensors[i].initialized = 0;
    tof_sensors[i].last_range_status = 0;
}

static void TOF_EnableSensor(uint8_t i)
{
    HAL_GPIO_WritePin(tof_sensors[i].xshut_port,
                      tof_sensors[i].xshut_pin,
                      GPIO_PIN_SET);
}

static void TOF_AllShutdown(void)
{
    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) TOF_DisableSensor(i);
    tof_init_status = 0;
}

/* ------------------------------------------------------------------ */
/* Pololu init() port                                                  */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef VL6180X_PololuInit(uint8_t dev7)
{
    uint8_t fresh = 0;
    if (readReg(dev7, SYSTEM__FRESH_OUT_OF_RESET, &fresh) != HAL_OK)
        return HAL_ERROR;

    if (fresh != 1) return HAL_OK;

    /* Mandatory: private registers (AN4545 SR03 settings, copied from
     * Pololu's library). */
    static const struct { uint16_t reg; uint8_t val; } seq[] =
    {
        {0x207, 0x01}, {0x208, 0x01}, {0x096, 0x00}, {0x097, 0xFD},
        {0x0E3, 0x01}, {0x0E4, 0x03}, {0x0E5, 0x02}, {0x0E6, 0x01},
        {0x0E7, 0x03}, {0x0F5, 0x02}, {0x0D9, 0x05}, {0x0DB, 0xCE},
        {0x0DC, 0x03}, {0x0DD, 0xF8}, {0x09F, 0x00}, {0x0A3, 0x3C},
        {0x0B7, 0x00}, {0x0BB, 0x3C}, {0x0B2, 0x09}, {0x0CA, 0x09},
        {0x198, 0x01}, {0x1B0, 0x17}, {0x1AD, 0x00}, {0x0FF, 0x05},
        {0x100, 0x05}, {0x199, 0x05}, {0x1A6, 0x1B}, {0x1AC, 0x3E},
        {0x1A7, 0x1F}, {0x030, 0x00}
    };

    for (uint16_t k = 0; k < sizeof(seq)/sizeof(seq[0]); k++)
    {
        if (writeReg(dev7, seq[k].reg, seq[k].val) != HAL_OK)
            return HAL_ERROR;
    }

    if (writeReg(dev7, SYSTEM__FRESH_OUT_OF_RESET, 0x00) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* Pololu configureDefault() port                                      */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef VL6180X_PololuConfigureDefault(uint8_t dev7)
{
    /* AN4545 Section 9 "Recommended public registers":
     * {0x0011} = 0x10 enables polling for 'New Sample ready' when
     * measurement completes. Pololu's library omits this; AN4545 Rev 2
     * lists it explicitly. Adding it for AN4545 compliance. */
    if (writeReg  (dev7, 0x0011,                            0x10)   != HAL_OK) return HAL_ERROR;

    if (writeReg  (dev7, READOUT__AVERAGING_SAMPLE_PERIOD,  0x30)   != HAL_OK) return HAL_ERROR;
    if (writeReg  (dev7, SYSALS__ANALOGUE_GAIN,             0x46)   != HAL_OK) return HAL_ERROR;
    if (writeReg  (dev7, SYSRANGE__VHV_REPEAT_RATE,         0xFF)   != HAL_OK) return HAL_ERROR;
    if (writeReg16(dev7, SYSALS__INTEGRATION_PERIOD,        0x0063) != HAL_OK) return HAL_ERROR;
    if (writeReg  (dev7, SYSRANGE__VHV_RECALIBRATE,         0x01)   != HAL_OK) return HAL_ERROR;

    if (writeReg  (dev7, SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09)   != HAL_OK) return HAL_ERROR;
    if (writeReg  (dev7, SYSALS__INTERMEASUREMENT_PERIOD,   0x31)   != HAL_OK) return HAL_ERROR;
    if (writeReg  (dev7, SYSTEM__INTERRUPT_CONFIG_GPIO,     0x24)   != HAL_OK) return HAL_ERROR;

    if (writeReg  (dev7, SYSRANGE__MAX_CONVERGENCE_TIME,    0x31)   != HAL_OK) return HAL_ERROR;
    if (writeReg  (dev7, INTERLEAVED_MODE__ENABLE,          0x00)   != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

static HAL_StatusTypeDef VL6180X_SetAddress(uint8_t old7, uint8_t new7)
{
    if (writeReg(old7, I2C_SLAVE__DEVICE_ADDRESS, new7 & 0x7F) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(TOF_ADDRESS_CHANGE_DELAY_MS);
    return HAL_OK;
}

static HAL_StatusTypeDef VL6180X_CheckModelID(uint8_t dev7)
{
    uint8_t id = 0;
    if (readReg(dev7, IDENTIFICATION__MODEL_ID, &id) != HAL_OK) return HAL_ERROR;
    if (id != VL6180X_MODEL_ID_VALUE)                          return HAL_ERROR;
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* Per-sensor init helper.                                             */
/*                                                                     */
/* Tries to bring up sensor i and move it to its unique address.       */
/* Returns 1 on success, 0 on any failure (with .init_fail_step set).  */
/* On failure, leaves XSHUT low so this sensor stays out of the bus.   */
/* ------------------------------------------------------------------ */

static uint8_t TOF_InitOne(uint8_t i)
{
    tof_sensors[i].init_fail_step = 0;
    tof_sensors[i].initialized    = 0;

    TOF_EnableSensor(i);
    HAL_Delay(TOF_BOOT_DELAY_MS);

    if (!TOF_AddressReady(ADDR8(VL6180X_DEFAULT_7BIT)))
    {
        tof_sensors[i].init_fail_step = 0x01;
        TOF_DisableSensor(i);
        return 0;
    }

    if (VL6180X_CheckModelID(VL6180X_DEFAULT_7BIT) != HAL_OK)
    {
        tof_sensors[i].init_fail_step = 0x02;
        TOF_DisableSensor(i);
        return 0;
    }

    if (VL6180X_PololuInit(VL6180X_DEFAULT_7BIT) != HAL_OK)
    {
        tof_sensors[i].init_fail_step = 0x07;
        TOF_DisableSensor(i);
        return 0;
    }

    if (VL6180X_PololuConfigureDefault(VL6180X_DEFAULT_7BIT) != HAL_OK)
    {
        tof_sensors[i].init_fail_step = 0x09;
        TOF_DisableSensor(i);
        return 0;
    }

    if (VL6180X_SetAddress(VL6180X_DEFAULT_7BIT,
                           tof_sensors[i].addr_7bit) != HAL_OK)
    {
        tof_sensors[i].init_fail_step = 0x03;
        TOF_DisableSensor(i);
        return 0;
    }

    HAL_Delay(TOF_ADDRESS_CHANGE_DELAY_MS);

    /* 0x29 must be empty now. If it isn't, another sensor is also awake
     * at 0x29 (e.g. a stuck-on rogue): we can't trust the address change
     * landed only on this sensor. Mark this slot failed and continue. */
    if (TOF_AddressReady(ADDR8(VL6180X_DEFAULT_7BIT)))
    {
        tof_sensors[i].init_fail_step = 0x04;
        TOF_DisableSensor(i);
        return 0;
    }

    if (!TOF_AddressReady(ADDR8(tof_sensors[i].addr_7bit)))
    {
        tof_sensors[i].init_fail_step = 0x05;
        TOF_DisableSensor(i);
        return 0;
    }

    if (VL6180X_CheckModelID(tof_sensors[i].addr_7bit) != HAL_OK)
    {
        tof_sensors[i].init_fail_step = 0x06;
        TOF_DisableSensor(i);
        return 0;
    }

    tof_sensors[i].initialized = 1;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Best-effort multi-sensor init.                                      */
/*                                                                     */
/* A failure on one sensor is recorded in that sensor's state, then    */
/* we move on to the next. Returns the bitmask of sensors that came    */
/* up successfully (0x0F if all four worked).                          */
/* ------------------------------------------------------------------ */

uint8_t TOF_Init(void)
{
    tof_pre_init_rogue = 0;

    TOF_AllShutdown();
    HAL_Delay(TOF_FULL_RESET_DELAY_MS);
    TOF_I2C_Recover();

    /* Informational only: with all XSHUT low, nobody should answer at 0x29.
     * If somebody does, one carrier board's SHDN circuit is broken. We
     * still try to init each sensor in case the others are healthy. */
    if (TOF_AddressReady(ADDR8(VL6180X_DEFAULT_7BIT)))
    {
        tof_pre_init_rogue = 1;
    }

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++)
    {
        (void)TOF_InitOne(i);  /* failures recorded per-sensor, ignored here */

        if (tof_sensors[i].initialized)
        {
            tof_init_status |= (1U << i);
        }

        HAL_Delay(20);
    }

    return tof_init_status;
}

/* ------------------------------------------------------------------ */
/* Read                                                                */
/* ------------------------------------------------------------------ */

uint8_t TOF_ReadDistance(uint8_t i)
{
    if (i >= TOF_SENSOR_COUNT)             return 0;
    if (tof_sensors[i].initialized == 0)   return 0;

    const uint8_t dev7 = tof_sensors[i].addr_7bit;

    if (writeReg(dev7, SYSRANGE__START, 0x01) != HAL_OK)
    {
        tof_sensors[i].last_range_status = 0xFE;
        return 0;
    }

    uint32_t t0  = HAL_GetTick();
    uint8_t  irq = 0;
    do
    {
        if (readReg(dev7, RESULT__INTERRUPT_STATUS_GPIO, &irq) != HAL_OK)
        {
            tof_sensors[i].last_range_status = 0xFD;
            return 0;
        }
        if ((HAL_GetTick() - t0) > TOF_RANGE_TIMEOUT_MS)
        {
            tof_sensors[i].last_range_status = 0xFC;
            (void)writeReg(dev7, SYSTEM__INTERRUPT_CLEAR, 0x07);
            return 0;
        }
    }
    while ((irq & 0x07U) != 0x04U);

    uint8_t rstat = 0;
    if (readReg(dev7, RESULT__RANGE_STATUS, &rstat) != HAL_OK)
    {
        tof_sensors[i].last_range_status = 0xFB;
        return 0;
    }
    tof_sensors[i].last_range_status = (uint8_t)(rstat >> 4);

    if ((rstat >> 4) != 0)
    {
        (void)writeReg(dev7, SYSTEM__INTERRUPT_CLEAR, 0x07);
        return 0;
    }

    uint8_t distance = 0;
    if (readReg(dev7, RESULT__RANGE_VAL, &distance) != HAL_OK)
    {
        tof_sensors[i].last_range_status = 0xFA;
        return 0;
    }

    (void)writeReg(dev7, SYSTEM__INTERRUPT_CLEAR, 0x07);
    return distance;
}

void TOF_ReadAll(volatile uint8_t distances[TOF_SENSOR_COUNT])
{
    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++)
    {
        /* Uninitialized sensors return 0 immediately, no bus traffic. */
        distances[i] = TOF_ReadDistance(i);
        HAL_Delay(TOF_BETWEEN_SENSOR_READ_DELAY_MS);
    }
}
