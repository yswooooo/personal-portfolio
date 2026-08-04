#include "app_encoder_speed.h"

#include "app_config.h"
#include <limits.h>
#include <stddef.h>

#define APP_ENCODER_TWO_PI  (6.28318530f)

static int64_t app_encoder_speed_get_delta(int32_t now_counts,
                                           int32_t last_counts)
{
    uint32_t wrapped_delta;

    wrapped_delta =
        (uint32_t)now_counts - (uint32_t)last_counts;

    if (wrapped_delta <= (uint32_t)INT32_MAX) {
        return (int64_t)wrapped_delta;
    }

    return (int64_t)wrapped_delta - (1LL << 32);
}

static void app_encoder_speed_save_sample(
    const app_encoder_sample_t *sample,
    app_encoder_speed_estimator_t *estimator)
{
    estimator->last_counts = sample->counts;
    estimator->last_timestamp_us = sample->timestamp_us;
    estimator->last_sequence = sample->sequence;
}

void app_encoder_speed_update(
    const app_encoder_sample_t *sample,
    app_encoder_speed_estimator_t *estimator,
    int8_t encoder_polarity)
{
    int64_t raw_delta_counts;

    if ((sample == NULL) || (estimator == NULL)) {
        return;
    }

    if (sample->sequence == estimator->last_sequence) {
        return;
    }

    if (estimator->initialized == 0u) {
        estimator->initialized = 1u;
        estimator->speed_valid = 0u;
        app_encoder_speed_save_sample(sample, estimator);
        return;
    }

    estimator->delta_time_us =
        sample->timestamp_us - estimator->last_timestamp_us;

    if (estimator->delta_time_us == 0u) {
        estimator->delta_counts = 0;
        estimator->speed_valid = 0u;
        app_encoder_speed_save_sample(sample, estimator);
        return;
    }

    raw_delta_counts =
        app_encoder_speed_get_delta(sample->counts,
                                    estimator->last_counts);
    estimator->delta_counts =
        raw_delta_counts * (int64_t)encoder_polarity;
    /*编码器Δ/s值*/
    estimator->speed_counts_per_s =
        (float)estimator->delta_counts * 1000000.0f
        / (float)estimator->delta_time_us;
    /*转子速度：未过减速箱*/
    estimator->motor_speed_rpm =
        estimator->speed_counts_per_s * 60.0f
        / LD2_ENCODER_COUNTS_PER_REV;
    /*轴(轮毂)速度：转子过减速比*/
    estimator->wheel_speed_rpm =
        estimator->motor_speed_rpm / APP_CHASSIS_GEAR_RATIO;
    /*轴(轮毂)线速度：过轮子半径*/
    estimator->wheel_speed_mps =
        estimator->wheel_speed_rpm * APP_ENCODER_TWO_PI
        * (APP_CHASSIS_WHEEL_RADIUS_MM / 1000.0f)
        / 60.0f;

    estimator->speed_valid = 1u;
    estimator->is_stale = 0u;
    app_encoder_speed_save_sample(sample, estimator);
}
