#ifndef TEST_APP_LD2RS_TASK_H
#define TEST_APP_LD2RS_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float motor_speed_rpm;
    float wheel_speed_rpm;
    float wheel_speed_mps;
} app_ld2rs_speed_feedback_t;

void app_ld2rs_task_run(void);
void app_ld2rs_task_init(void);
bool app_ld2rs_task_get_speed_feedback(
    uint8_t motor_number,
    app_ld2rs_speed_feedback_t *feedback);

#endif
