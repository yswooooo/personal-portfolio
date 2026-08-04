#ifndef APP_ENCODER_SPEED_H
#define APP_ENCODER_SPEED_H

#include <stdint.h>

typedef struct
{
    int32_t counts;
    uint64_t timestamp_us;
    uint32_t sequence;
} app_encoder_sample_t;

typedef struct
{
    uint8_t initialized;
    uint8_t speed_valid;
    uint8_t is_stale;

    int32_t last_counts;
    uint64_t last_timestamp_us;
    uint32_t last_sequence;

    int64_t delta_counts;
    uint64_t delta_time_us;
    float speed_counts_per_s;
    float motor_speed_rpm;
    float wheel_speed_rpm;
    float wheel_speed_mps;
} app_encoder_speed_estimator_t;

void app_encoder_speed_update(
    const app_encoder_sample_t *sample,
    app_encoder_speed_estimator_t *estimator,
    int8_t encoder_polarity);

#endif /* APP_ENCODER_SPEED_H */
