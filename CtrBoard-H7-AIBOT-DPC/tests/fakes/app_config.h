#ifndef TEST_APP_CONFIG_H
#define TEST_APP_CONFIG_H

#define APP_LD2_MOTOR_COUNT                  2u
#define APP_LD2_MOTOR_MAX_COUNT              3u
#define APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT  3u
#define APP_MODBUS_RTU_INTERFRAME_MS         1u
#define APP_CHASSIS_SPEED_LIMIT_RPM          4500
#define APP_CHASSIS_MAX_LINEAR_VEL_MPS       1.0f
#define APP_CHASSIS_MAX_ANGULAR_VEL_RPS      1.0f
#define APP_CHASSIS_WHEEL_RADIUS_MM          75.0f
#define APP_CHASSIS_GEAR_RATIO               20.0f
#define LD2_ENCODER_COUNTS_PER_REV            131072.0f
#define APP_LD2_ENCODER_POLARITY_M1          1
#define APP_LD2_ENCODER_POLARITY_M2          (-1)

#endif
