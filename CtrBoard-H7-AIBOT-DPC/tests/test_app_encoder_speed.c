#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_encoder_speed.h"

static void expect_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static void expect_near(float actual,
                        float expected,
                        float tolerance,
                        const char *message)
{
    float difference = actual - expected;

    if (difference < 0.0f) {
        difference = -difference;
    }

    if (difference > tolerance) {
        fprintf(stderr,
                "FAIL: %s (actual=%.9f expected=%.9f)\n",
                message,
                (double)actual,
                (double)expected);
        exit(1);
    }
}

static void test_first_sample_only_establishes_baseline(void)
{
    app_encoder_sample_t sample = {1000, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, 1);

    expect_true(estimator.initialized == 1u,
                "first sample must initialize the estimator");
    expect_true(estimator.speed_valid == 0u,
                "first sample must not produce a speed");
    expect_true(estimator.last_counts == 1000,
                "first sample must save the position baseline");
    expect_true(estimator.last_timestamp_us == 100000u,
                "first sample must save the timestamp baseline");
    expect_true(estimator.last_sequence == 1u,
                "first sample must consume its sequence");
}

static void test_second_sample_calculates_all_speed_units(void)
{
    app_encoder_sample_t sample = {1000, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, 1);

    sample.counts = 1120;
    sample.timestamp_us = 110000u;
    sample.sequence = 2u;
    app_encoder_speed_update(&sample, &estimator, 1);

    expect_true(estimator.speed_valid == 1u,
                "second sample must produce a valid speed");
    expect_true(estimator.delta_counts == 120,
                "second sample must preserve the signed count delta");
    expect_true(estimator.delta_time_us == 10000u,
                "second sample must preserve the DWT interval");
    expect_near(estimator.speed_counts_per_s,
                12000.0f,
                0.01f,
                "counts per second");
    expect_near(estimator.motor_speed_rpm,
                5.493164063f,
                0.00001f,
                "motor rpm");
    expect_near(estimator.wheel_speed_rpm,
                0.274658203f,
                0.00001f,
                "wheel rpm");
    expect_near(estimator.wheel_speed_mps,
                0.002157160f,
                0.000001f,
                "wheel linear speed");
}

static void test_negative_motion_preserves_direction(void)
{
    app_encoder_sample_t sample = {1120, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, 1);

    sample.counts = 1000;
    sample.timestamp_us = 110000u;
    sample.sequence = 2u;
    app_encoder_speed_update(&sample, &estimator, 1);

    expect_true(estimator.delta_counts == -120,
                "decreasing encoder position must produce a negative delta");
    expect_near(estimator.motor_speed_rpm,
                -5.493164063f,
                0.00001f,
                "negative motor rpm");
}

static void test_encoder_polarity_reverses_direction(void)
{
    app_encoder_sample_t sample = {1000, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, -1);

    sample.counts = 1120;
    sample.timestamp_us = 110000u;
    sample.sequence = 2u;
    app_encoder_speed_update(&sample, &estimator, -1);

    expect_true(estimator.delta_counts == -120,
                "negative polarity must reverse the count delta");
    expect_near(estimator.motor_speed_rpm,
                -5.493164063f,
                0.00001f,
                "negative polarity motor rpm");
}

static void test_zero_time_difference_is_consumed_without_division(void)
{
    app_encoder_sample_t sample = {1000, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, 1);

    sample.counts = 1120;
    sample.sequence = 2u;
    app_encoder_speed_update(&sample, &estimator, 1);

    expect_true(estimator.speed_valid == 0u,
                "zero time difference must leave speed invalid");
    expect_true(estimator.last_counts == 1120,
                "zero time difference must still update the baseline");
    expect_true(estimator.last_sequence == 2u,
                "zero time difference must consume the sample");
}

static void test_duplicate_sequence_is_not_processed_twice(void)
{
    app_encoder_sample_t sample = {1000, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;
    float previous_speed;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, 1);

    sample.counts = 1120;
    sample.timestamp_us = 110000u;
    sample.sequence = 2u;
    app_encoder_speed_update(&sample, &estimator, 1);
    previous_speed = estimator.motor_speed_rpm;

    sample.counts = 5000;
    sample.timestamp_us = 120000u;
    app_encoder_speed_update(&sample, &estimator, 1);

    expect_near(estimator.motor_speed_rpm,
                previous_speed,
                0.0f,
                "duplicate sequence must preserve the last speed");
    expect_true(estimator.last_counts == 1120,
                "duplicate sequence must preserve the last baseline");
}

static void test_forward_32bit_position_wrap_is_one_count(void)
{
    app_encoder_sample_t sample = {INT32_MAX, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, 1);

    sample.counts = INT32_MIN;
    sample.timestamp_us = 1100000u;
    sample.sequence = 2u;
    app_encoder_speed_update(&sample, &estimator, 1);

    expect_true(estimator.delta_counts == 1,
                "forward signed position wrap must equal one count");
}

static void test_reverse_32bit_position_wrap_is_minus_one_count(void)
{
    app_encoder_sample_t sample = {INT32_MIN, 100000u, 1u};
    app_encoder_speed_estimator_t estimator;

    memset(&estimator, 0, sizeof(estimator));
    app_encoder_speed_update(&sample, &estimator, 1);

    sample.counts = INT32_MAX;
    sample.timestamp_us = 1100000u;
    sample.sequence = 2u;
    app_encoder_speed_update(&sample, &estimator, 1);

    expect_true(estimator.delta_counts == -1,
                "reverse signed position wrap must equal minus one count");
}

int main(void)
{
    test_first_sample_only_establishes_baseline();
    test_second_sample_calculates_all_speed_units();
    test_negative_motion_preserves_direction();
    test_encoder_polarity_reverses_direction();
    test_zero_time_difference_is_consumed_without_division();
    test_duplicate_sequence_is_not_processed_twice();
    test_forward_32bit_position_wrap_is_one_count();
    test_reverse_32bit_position_wrap_is_minus_one_count();
    puts("PASS: encoder speed estimator");
    return 0;
}
