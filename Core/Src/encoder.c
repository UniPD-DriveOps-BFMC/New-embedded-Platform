#include "encoder.h"
#include <math.h>


static TIM_HandleTypeDef *enc_tim = 0;

static volatile Encoder_State_t enc_state = {0};

static volatile int32_t encoder_offset_counts = 0;
static float previous_velocity_mps = 0.0f;
static float previous_distance_m = 0.0f;

static int16_t previous_counter = 0;

static float Encoder_CountsToMeters(int32_t counts)
{
    float rev_factor =
        ((float)counts * (float)GEAR_NUM) /
        ((float)ENCODER_CPR * (float)GEAR_DEN);

    return rev_factor * TWO_PI * WHEEL_RADIUS;
}

static int32_t Encoder_ModNearest(int32_t value, int32_t modulo)
{
    int32_t r = value % modulo;

    if (r > modulo / 2)
        r -= modulo;

    if (r < -modulo / 2)
        r += modulo;

    return r;
}

void Encoder_Init(TIM_HandleTypeDef *htim)
{
    enc_tim = htim;

    HAL_TIM_Encoder_Start(enc_tim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(enc_tim, 0);
    previous_counter = 0;

    enc_state.delta_counts = 0;
    enc_state.total_counts_raw = 0;
    enc_state.total_counts_corrected = 0;

    enc_state.distance_m = 0.0f;
    enc_state.distance_m_filtered = 0.0f;
    enc_state.velocity_mps = 0.0f;
    enc_state.velocity_mps_filtered = 0.0f;
    enc_state.acceleration_mps2 = 0.0f;

    enc_state.z_seen = 0;
    enc_state.z_count = 0;

    encoder_offset_counts = 0;
    previous_velocity_mps = 0.0f;
    previous_distance_m = 0.0f;
}

void Encoder_Update_ISR(void)
{
    int16_t delta;
    int32_t corrected_counts;
    float distance_now;

    int16_t current_counter = (int16_t)__HAL_TIM_GET_COUNTER(enc_tim);
    delta = current_counter - previous_counter;
    previous_counter = current_counter;

    if (delta >= -1 && delta <= 1)
    {
        delta = 0;
    }

    enc_state.delta_counts = delta;
    enc_state.total_counts_raw += delta;

    corrected_counts = enc_state.total_counts_raw - encoder_offset_counts;
    enc_state.total_counts_corrected = corrected_counts;

    distance_now = Encoder_CountsToMeters(corrected_counts);

    enc_state.velocity_mps =
        (distance_now - previous_distance_m) / ENCODER_TS_S;

    if (fabsf(enc_state.velocity_mps) < 1e-10f)
    {
        enc_state.velocity_mps = 0.0f;
    }

    enc_state.velocity_mps_filtered =
        (VELOCITY_LPF_ALPHA * enc_state.velocity_mps) +
        ((1.0f - VELOCITY_LPF_ALPHA) * enc_state.velocity_mps_filtered);

    if (fabsf(enc_state.velocity_mps_filtered) < 1e-10f)
    {
        enc_state.velocity_mps_filtered = 0.0f;
    }

    enc_state.distance_m_filtered += enc_state.velocity_mps_filtered * ENCODER_TS_S;

    enc_state.acceleration_mps2 =
        (enc_state.velocity_mps_filtered - previous_velocity_mps) /
        ENCODER_TS_S;

    enc_state.distance_m = distance_now;

    previous_distance_m = distance_now;
    previous_velocity_mps = enc_state.velocity_mps_filtered;
}

void Encoder_Z_ISR(void)
{
    int32_t corrected_counts =
        enc_state.total_counts_raw - encoder_offset_counts;

    int32_t residual =
        Encoder_ModNearest(corrected_counts, ENCODER_CPR);

    int32_t correction =
        (int32_t)(Z_CORRECTION_GAIN * (float)residual);

    encoder_offset_counts += correction;

    enc_state.z_seen = 1;
    enc_state.z_count++;
}

Encoder_State_t Encoder_GetState(void)
{
    return enc_state;
}
