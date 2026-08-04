#ifndef TEST_APP_CHASSIS_MOTOR_CTRL_H
#define TEST_APP_CHASSIS_MOTOR_CTRL_H

#include <stdint.h>
#include "ld2_motor.h"
#include "bsp_rc.h"

typedef struct {
    ld2_motor_handle_t *ld2_motor;
    uint32_t last_speed_update_tick_ms;
    uint32_t last_monitor_tick_ms;
    int16_t ref_speed_rpm;
    uint8_t motor_id;
    uint8_t is_speed_mode_ok;
    uint8_t is_zero_speed_ok;
    uint8_t is_speed_source_ok;
    uint8_t is_accel_decel_ok;
} ld2rs_motor_ctrl_t;

typedef struct {
    float m1_ref_speed_rpm;
    float m1_feedback_speed_rpm;
    float m1_speed_error_rpm;
    float m1_read_rtt_ms;
    float m1_write_rtt_ms;
    float m1_cycle_ms;
    float m2_ref_speed_rpm;
    float m2_feedback_speed_rpm;
    float m2_speed_error_rpm;
    float m2_read_rtt_ms;
    float m2_write_rtt_ms;
    float m2_cycle_ms;
    float m1_offline_total_count;
    float m1_offline_confirmed;
    float m2_offline_total_count;
    float m2_offline_confirmed;
    float m1_encoder_position_counts;
    float m2_encoder_position_counts;
} vofa_motor_info_t;

extern vofa_motor_info_t g_vofa_speed;
extern ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m1;
extern ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m2;

void app_rc_channels_check_lost(RC_Channels_t *channels);
void app_diff_chassis_motor_ctrl_rc_map_to_chassis(const RC_Filter_t *filter,
                                                    RC_ChassisCmd_t *command);
void app_diff_drive_compute(const float *linear_velocity_mps,
                            const float *angular_velocity_radps,
                            int16_t *left_rpm_out,
                            int16_t *right_rpm_out);

#endif
