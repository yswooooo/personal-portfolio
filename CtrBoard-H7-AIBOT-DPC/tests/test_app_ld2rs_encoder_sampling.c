#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_encoder_speed.h"
#include "../App/Src/app_ld2rs_task.c"

bsp_rs485_handle_t g_rs485_bus;
vofa_motor_info_t g_vofa_speed;
ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m1;
ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m2;
ld2_motor_handle_t g_ld2rs_dev_m1;
ld2_motor_handle_t g_ld2rs_dev_m2;
RC_Channels_t g_rc;
RC_Filter_t g_rc_filter;
RC_ChassisCmd_t g_rc_chassis;
volatile uint8_t g_emergency_stop_flag;

static uint32_t s_tick;
static uint16_t s_crc_result;

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

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

uint64_t BSP_DWT_GetTickUs(void)
{
    return (uint64_t)s_tick * 1000u;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart_handle,
                                       uint8_t *buffer,
                                       uint16_t length)
{
    (void)uart_handle;
    (void)buffer;
    (void)length;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart_handle)
{
    (void)uart_handle;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart_handle)
{
    (void)uart_handle;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *uart_handle,
                                              uint8_t *buffer,
                                              uint16_t capacity)
{
    (void)uart_handle;
    (void)buffer;
    (void)capacity;
    return HAL_OK;
}

uint16_t modbus_rtu_crc16(const uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
    return s_crc_result;
}

bsp_rs485_status_t bsp_rs485_start_tx(bsp_rs485_handle_t *bus,
                                      const uint8_t *tx_buffer,
                                      uint16_t tx_len,
                                      uint8_t *rx_buffer,
                                      uint16_t rx_capacity,
                                      uint32_t timeout_ms)
{
    (void)tx_buffer;
    (void)tx_len;
    (void)rx_buffer;
    (void)rx_capacity;
    (void)timeout_ms;
    bus->state = BSP_RS485_STATE_TX_BUSY;
    return BSP_RS485_STATUS_OK;
}

bsp_rs485_state_t bsp_rs485_poll(bsp_rs485_handle_t *bus)
{
    return bus->state;
}

void bsp_rs485_ack_done(bsp_rs485_handle_t *bus)
{
    bus->state = BSP_RS485_STATE_IDLE;
}

void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus)
{
    bus->state = BSP_RS485_STATE_IDLE;
}

void app_rc_channels_check_lost(RC_Channels_t *channels)
{
    (void)channels;
}

void app_diff_chassis_motor_ctrl_rc_map_to_chassis(
    const RC_Filter_t *filter,
    RC_ChassisCmd_t *command)
{
    (void)filter;
    (void)command;
}

void app_diff_drive_compute(const float *linear_velocity_mps,
                            const float *angular_velocity_radps,
                            int16_t *left_rpm_out,
                            int16_t *right_rpm_out)
{
    (void)linear_velocity_mps;
    (void)angular_velocity_radps;
    *left_rpm_out = 0;
    *right_rpm_out = 0;
}

static void reset_local_fsm(motor_fsm_t *fsm,
                            ld2rs_motor_ctrl_t *motor_ctrl,
                            ld2_motor_handle_t *motor_dev)
{
    memset(&g_rs485_bus, 0, sizeof(g_rs485_bus));
    memset(&g_vofa_speed, 0, sizeof(g_vofa_speed));
    memset(&g_rc, 0, sizeof(g_rc));
    memset(fsm, 0, sizeof(*fsm));
    memset(motor_ctrl, 0, sizeof(*motor_ctrl));
    memset(motor_dev, 0, sizeof(*motor_dev));

    motor_ctrl->motor_id = 1u;
    motor_dev->slave_id = 1u;
    motor_dev->timeout_ms = 100u;
    g_rc.sw_st[eRC_SW_A].curr = eRC_POS_MID;
    g_rs485_bus.state = BSP_RS485_STATE_IDLE;
    s_tick = 20u;
    s_crc_result = 0u;
    g_emergency_stop_flag = 0u;

    app_motor_fsm_init(fsm, motor_ctrl, motor_dev);
}

static void fill_encoder_response(motor_fsm_t *fsm,
                                  uint32_t position_raw,
                                  uint64_t timestamp_us)
{
    g_rs485_bus.rx_len = 9u;
    g_rs485_bus.rx_timestamp_us = timestamp_us;
    fsm->rx_frame[0] = fsm->motor_dev->slave_id;
    fsm->rx_frame[1] = MODBUS_RTU_FC_READ_HOLDING;
    fsm->rx_frame[2] = 4u;
    fsm->rx_frame[3] = (uint8_t)(position_raw >> 24);
    fsm->rx_frame[4] = (uint8_t)(position_raw >> 16);
    fsm->rx_frame[5] = (uint8_t)(position_raw >> 8);
    fsm->rx_frame[6] = (uint8_t)position_raw;
}

static void test_valid_encoder_response_publishes_complete_sample(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;

    reset_local_fsm(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
    fill_encoder_response(&fsm, 131072u, 500000u);

    app_motor_fsm_step(&fsm, BSP_RS485_STATE_DONE);

    expect_true(fsm.encoder_sample.counts == 131072,
                "valid PrB.24 must publish the parsed count");
    expect_true(fsm.encoder_sample.timestamp_us == 500000u,
                "sample must use the matching BSP timestamp");
    expect_true(fsm.encoder_sample.sequence == 1u,
                "valid PrB.24 must publish one sequence");
}

static void test_invalid_encoder_response_does_not_publish(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;

    reset_local_fsm(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
    fill_encoder_response(&fsm, 131072u, 500000u);
    s_crc_result = 1u;

    app_motor_fsm_step(&fsm, BSP_RS485_STATE_DONE);

    expect_true(fsm.encoder_sample.sequence == 0u,
                "invalid CRC must not publish an encoder sample");
}

static void test_encoder_timeout_does_not_publish(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;

    reset_local_fsm(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;

    app_motor_fsm_step(&fsm, BSP_RS485_STATE_TIMEOUT);

    expect_true(fsm.encoder_sample.sequence == 0u,
                "timeout must not publish an encoder sample");
}

static void test_structure_publication_and_motor_isolation(void)
{
    app_encoder_sample_t new_sample;
    app_ld2rs_speed_feedback_t m1_feedback;
    app_ld2rs_speed_feedback_t m2_feedback;

    memset(&g_ld2rs_motor_ctrl_m1, 0, sizeof(g_ld2rs_motor_ctrl_m1));
    memset(&g_ld2rs_motor_ctrl_m2, 0, sizeof(g_ld2rs_motor_ctrl_m2));
    memset(&g_ld2rs_dev_m1, 0, sizeof(g_ld2rs_dev_m1));
    memset(&g_ld2rs_dev_m2, 0, sizeof(g_ld2rs_dev_m2));
    g_ld2rs_motor_ctrl_m1.motor_id = 1u;
    g_ld2rs_motor_ctrl_m2.motor_id = 2u;
    app_motor_fsm_init(&s_motor_fsm_m1,
                       &g_ld2rs_motor_ctrl_m1,
                       &g_ld2rs_dev_m1);
    app_motor_fsm_init(&s_motor_fsm_m2,
                       &g_ld2rs_motor_ctrl_m2,
                       &g_ld2rs_dev_m2);

    new_sample.counts = 1000;
    new_sample.timestamp_us = 100000u;
    new_sample.sequence = 1u;
    app_encoder_sample_publish(
        &s_motor_fsm_m1.encoder_sample,
        &new_sample);

    new_sample.counts = 2000;
    app_encoder_sample_publish(
        &s_motor_fsm_m2.encoder_sample,
        &new_sample);

    app_encoder_speed_update(
        &s_motor_fsm_m1.encoder_sample,
        &s_motor_fsm_m1.speed_estimator,
        APP_LD2_ENCODER_POLARITY_M1);
    app_encoder_speed_update(
        &s_motor_fsm_m2.encoder_sample,
        &s_motor_fsm_m2.speed_estimator,
        APP_LD2_ENCODER_POLARITY_M2);

    new_sample.counts = 1120;
    new_sample.timestamp_us = 110000u;
    new_sample.sequence = 2u;
    app_encoder_sample_publish(
        &s_motor_fsm_m1.encoder_sample,
        &new_sample);

    new_sample.counts = 1880;
    app_encoder_sample_publish(
        &s_motor_fsm_m2.encoder_sample,
        &new_sample);

    app_encoder_speed_update(
        &s_motor_fsm_m1.encoder_sample,
        &s_motor_fsm_m1.speed_estimator,
        APP_LD2_ENCODER_POLARITY_M1);
    app_encoder_speed_update(
        &s_motor_fsm_m2.encoder_sample,
        &s_motor_fsm_m2.speed_estimator,
        APP_LD2_ENCODER_POLARITY_M2);

    expect_true(app_ld2rs_task_get_speed_feedback(1u, &m1_feedback),
                "M1 getter must return a valid result");
    expect_true(app_ld2rs_task_get_speed_feedback(2u, &m2_feedback),
                "M2 getter must return a valid result");
    expect_near(m1_feedback.motor_speed_rpm,
                5.493164063f,
                0.00001f,
                "M1 motor rpm");
    expect_near(m2_feedback.motor_speed_rpm,
                5.493164063f,
                0.00001f,
                "M2 independent polarity");
    expect_near(m1_feedback.wheel_speed_rpm,
                0.274658203f,
                0.00001f,
                "M1 wheel rpm");
    expect_near(m1_feedback.wheel_speed_mps,
                0.002157160f,
                0.000001f,
                "M1 wheel linear speed");
    expect_true(!app_ld2rs_task_get_speed_feedback(0u, &m1_feedback),
                "invalid motor number must be rejected");
    expect_true(!app_ld2rs_task_get_speed_feedback(1u, NULL),
                "null feedback pointer must be rejected");
}

int main(void)
{
    test_valid_encoder_response_publishes_complete_sample();
    test_invalid_encoder_response_does_not_publish();
    test_encoder_timeout_does_not_publish();
    test_structure_publication_and_motor_isolation();
    puts("PASS: LD2RS encoder sample publication");
    return 0;
}
