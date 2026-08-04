/**
  ******************************************************************************
  * @file    ld2rs_task.h
  * @brief   非阻塞 LD2-RS 电机控制任务
  *
  * @details 一条 app_ld2rs_task_run() 完成 RC→差速→M1/M2 Modbus 状态机。
  *          主循环中直接调用, 零 CPU 死等。
  ******************************************************************************
  */

#ifndef APP_LD2RS_TASK_H
#define APP_LD2RS_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions --------------------------------------------------------*/

/* ------------------------------------------------------------------ */
/* 静态实例                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    float motor_speed_rpm;
    float wheel_speed_rpm;
    float wheel_speed_mps;
} app_ld2rs_speed_feedback_t;

void app_ld2rs_task_init(void);
void app_ld2rs_task_run(void);
bool app_ld2rs_task_get_speed_feedback(
    uint8_t motor_number,
    app_ld2rs_speed_feedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif /* APP_LD2RS_TASK_H */


