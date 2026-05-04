#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define ENCODER_CPR          8192
#define GEAR_NUM             66
#define GEAR_DEN             527
#define WHEEL_RADIUS       	 0.03224f

/* Your existing TIM11 is 1 ms, so use 0.001 s */
#define ENCODER_TS_S         0.001f

#define VELOCITY_LPF_ALPHA   0.10f
#define Z_CORRECTION_GAIN    0.20f

#define TWO_PI 6.28318530718f

#define TRANSMISSION_RATIO 		((float)GEAR_DEN / (float)GEAR_NUM)

typedef struct
{
    int16_t delta_counts;
    int32_t total_counts_raw;
    int32_t total_counts_corrected;

    float distance_m;
    float distance_m_filtered;
    float velocity_mps;
    float velocity_mps_filtered;
    float acceleration_mps2;

    uint8_t z_seen;
    uint32_t z_count;
} Encoder_State_t;

void Encoder_Init(TIM_HandleTypeDef *htim);
void Encoder_Update_ISR(void);
void Encoder_Z_ISR(void);

Encoder_State_t Encoder_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
