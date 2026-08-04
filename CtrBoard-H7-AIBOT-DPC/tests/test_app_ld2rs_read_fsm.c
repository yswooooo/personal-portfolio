#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
static bsp_rs485_status_t s_start_status;
static unsigned int s_start_calls;
static unsigned int s_ack_calls;
static unsigned int s_cancel_calls;

static void expect_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
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
    return 0u;
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
    s_start_calls++;
    if (s_start_status == BSP_RS485_STATUS_OK) {
        bus->state = BSP_RS485_STATE_TX_BUSY;
    }
    return s_start_status;
}

bsp_rs485_state_t bsp_rs485_poll(bsp_rs485_handle_t *bus)
{
    return bus->state;
}

void bsp_rs485_ack_done(bsp_rs485_handle_t *bus)
{
    s_ack_calls++;
    bus->state = BSP_RS485_STATE_IDLE;
    bus->rx_error_code = 0u;
    bus->state_error_code = 0u;
}

void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus)
{
    s_cancel_calls++;
    bus->state = BSP_RS485_STATE_IDLE;
    bus->rx_error_code = 0u;
    bus->state_error_code = 0u;
}

void app_rc_channels_check_lost(RC_Channels_t *channels)
{
    (void)channels;
}

void app_diff_chassis_motor_ctrl_rc_map_to_chassis(const RC_Filter_t *filter,
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

static void reset_fixture(motor_fsm_t *fsm,
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
    s_start_status = BSP_RS485_STATUS_OK;
    s_start_calls = 0u;
    s_ack_calls = 0u;
    s_cancel_calls = 0u;
    g_emergency_stop_flag = 0u;

    app_motor_fsm_init(fsm, motor_ctrl, motor_dev);
}

static void test_status_success_selects_encoder_transaction(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_STATUS_SPEED;
    fsm.tx_start_tick_ms = 10u;
    g_rs485_bus.rx_len = 9u;
    fsm.rx_frame[0] = 1u;
    fsm.rx_frame[1] = MODBUS_RTU_FC_READ_HOLDING;
    fsm.rx_frame[2] = 4u;
    fsm.rx_frame[3] = 0x00u;
    fsm.rx_frame[4] = 0x03u;
    fsm.rx_frame[5] = 0x00u;
    fsm.rx_frame[6] = 0x7Bu;

    result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_DONE);

    expect_true(result == APP_MOTOR_FSM_STEP_DONE_RELEASED,
                "status success must release the completed transaction");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_READ_REQ,
                "status success must continue through the common read request");
    expect_true(fsm.read_transaction
                    == APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION,
                "status success must select the encoder sub-transaction");
    expect_true(fsm.read_retry_count == 0u,
                "status success must reset the retry budget for encoder");
    expect_true(g_vofa_speed.m1_feedback_speed_rpm == 123.0f,
                "status success must preserve feedback speed parsing");
    expect_true(s_ack_calls == 1u,
                "status success must acknowledge the bus exactly once");
}

static void test_short_status_response_consumes_retry(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_STATUS_SPEED;
    g_rs485_bus.rx_len = 7u;
    fsm.rx_frame[0] = 1u;
    fsm.rx_frame[1] = MODBUS_RTU_FC_READ_HOLDING;
    fsm.rx_frame[2] = 2u;
    fsm.rx_frame[3] = 0x00u;
    fsm.rx_frame[4] = 0x03u;

    result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_DONE);

    expect_true(result == APP_MOTOR_FSM_STEP_DONE_RELEASED,
                "malformed status response must release the completed bus");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_READ_REQ,
                "malformed status response must retry through common READ_REQ");
    expect_true(fsm.read_transaction
                    == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED,
                "malformed status response must not advance to encoder");
    expect_true(fsm.read_retry_count == 1u,
                "malformed status response must consume one APP retry");
    expect_true(g_vofa_speed.m1_feedback_speed_rpm == 0.0f,
                "malformed status response must not update feedback speed");
    expect_true(s_ack_calls == 1u,
                "malformed status response must acknowledge the bus once");
}

static void test_encoder_start_failure_exhaustion_reaches_write_path(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;
    unsigned int attempt;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_READ_REQ;
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
    fsm.encoder_position_counts = 77;
    g_vofa_speed.m1_encoder_position_counts = 77.0f;
    s_start_status = BSP_RS485_STATUS_ERR_HAL;

    for (attempt = 1u; attempt <= 3u; attempt++) {
        result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_IDLE);
        expect_true(fsm.read_retry_count == attempt,
                    "encoder start error must consume one APP retry");
        if (attempt < 3u) {
            expect_true(result == APP_MOTOR_FSM_STEP_ERROR_RELEASED,
                        "retryable encoder start error must yield the scheduler");
            expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_READ_REQ,
                        "retryable encoder start error must remain in common READ_REQ");
        }
    }

    expect_true(result == APP_MOTOR_FSM_STEP_NONE,
                "encoder start exhaustion must keep this motor for its write");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_READ_DONE,
                "encoder start exhaustion must continue to READ_DONE");
    expect_true(fsm.force_zero_speed == 0u,
                "encoder start exhaustion must not force zero speed");
    expect_true(fsm.encoder_position_counts == 77
                    && g_vofa_speed.m1_encoder_position_counts == 77.0f,
                "encoder start exhaustion must preserve the previous value");
}

static void test_encoder_timeout_has_three_app_attempts(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;
    unsigned int attempt;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
    fsm.encoder_position_counts = -9;
    g_vofa_speed.m1_encoder_position_counts = -9.0f;

    for (attempt = 1u; attempt <= 3u; attempt++) {
        fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
        result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_TIMEOUT);
        expect_true(fsm.read_retry_count == attempt,
                    "encoder timeout must consume one APP retry");
        if (attempt < 3u) {
            expect_true(result == APP_MOTOR_FSM_STEP_ERROR_RELEASED,
                        "retryable encoder timeout must release the scheduler");
            expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_READ_REQ,
                        "retryable encoder timeout must return to common READ_REQ");
        }
    }

    expect_true(result == APP_MOTOR_FSM_STEP_NONE,
                "encoder timeout exhaustion must prioritize speed write");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_READ_DONE,
                "encoder timeout exhaustion must leave the read transaction");
    expect_true(fsm.offline_total_count == 0u,
                "encoder timeout must not affect offline statistics");
    expect_true(fsm.force_zero_speed == 0u,
                "encoder timeout must not force zero speed");
    expect_true(fsm.encoder_position_counts == -9
                    && g_vofa_speed.m1_encoder_position_counts == -9.0f,
                "encoder timeout must preserve the last valid value");
    expect_true(s_ack_calls == 3u,
                "every encoder timeout attempt must acknowledge the bus");
}

static void test_status_timeout_preserves_original_force_zero_policy(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result = APP_MOTOR_FSM_STEP_NONE;
    unsigned int attempt;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_STATUS_SPEED;

    for (attempt = 1u; attempt <= 3u; attempt++) {
        fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
        result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_TIMEOUT);
    }

    expect_true(result == APP_MOTOR_FSM_STEP_ERROR_RELEASED,
                "status timeout exhaustion must release the scheduler");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_WRITE_REQ,
                "status timeout exhaustion must continue to speed write");
    expect_true(fsm.force_zero_speed == 1u,
                "status timeout exhaustion must preserve forced zero speed");
    expect_true(fsm.offline_total_count == 3u
                    && fsm.is_offline_confirmed == 1u,
                "status timeouts must preserve offline accounting");
}

static void test_safety_stop_cancels_encoder_wait(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
    g_rs485_bus.state = BSP_RS485_STATE_RX_WAIT;
    g_emergency_stop_flag = 1u;

    result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_RX_WAIT);

    expect_true(result == APP_MOTOR_FSM_STEP_NONE,
                "safety cancellation must keep this motor for zero-speed write");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_READ_DONE,
                "safety cancellation must leave the encoder observation");
    expect_true(s_cancel_calls == 1u,
                "safety cancellation must use the BSP cancellation boundary");
    expect_true(g_rs485_bus.state == BSP_RS485_STATE_IDLE,
                "safety cancellation must release the bus");
}

static void test_unknown_read_transaction_forces_zero_without_starting_read(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_READ_REQ;
    fsm.read_transaction = (motor_read_transaction_t)99;

    result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_IDLE);

    expect_true(s_start_calls == 0u,
                "unknown read transaction must not start a Modbus read");
    expect_true(result == APP_MOTOR_FSM_STEP_NONE,
                "unknown read transaction must keep this motor for zero-speed write");
    expect_true(fsm.force_zero_speed == 1u,
                "unknown read transaction must force zero speed");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_WRITE_REQ,
                "unknown read transaction must enter WRITE_REQ directly");
}

static void test_unknown_read_transaction_does_not_parse_completed_response(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = (motor_read_transaction_t)99;
    fsm.encoder_position_counts = 77;
    g_vofa_speed.m1_encoder_position_counts = 77.0f;
    g_rs485_bus.rx_len = 9u;
    fsm.rx_frame[0] = 1u;
    fsm.rx_frame[1] = MODBUS_RTU_FC_READ_HOLDING;
    fsm.rx_frame[2] = 4u;
    fsm.rx_frame[3] = 0x12u;
    fsm.rx_frame[4] = 0x34u;
    fsm.rx_frame[5] = 0x56u;
    fsm.rx_frame[6] = 0x78u;

    result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_DONE);

    expect_true(result == APP_MOTOR_FSM_STEP_NONE,
                "unknown completed read must keep this motor for zero-speed write");
    expect_true(fsm.encoder_position_counts == 77
                    && g_vofa_speed.m1_encoder_position_counts == 77.0f,
                "unknown completed read must not parse a legal response format");
    expect_true(fsm.force_zero_speed == 1u,
                "unknown completed read must force zero speed");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_WRITE_REQ,
                "unknown completed read must enter WRITE_REQ directly");
    expect_true(s_ack_calls == 1u,
                "unknown completed read must acknowledge the bus exactly once");
}

static void test_unknown_read_transaction_failure_forces_zero_without_retry(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
    fsm.read_transaction = (motor_read_transaction_t)99;

    result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_TIMEOUT);

    expect_true(result == APP_MOTOR_FSM_STEP_NONE,
                "unknown failed read must keep this motor for zero-speed write");
    expect_true(fsm.force_zero_speed == 1u,
                "unknown failed read must force zero speed");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_WRITE_REQ,
                "unknown failed read must enter WRITE_REQ without retry");
    expect_true(s_ack_calls == 1u,
                "unknown failed read must acknowledge the bus exactly once");
}

int main(void)
{
    test_status_success_selects_encoder_transaction();
    test_short_status_response_consumes_retry();
    test_encoder_start_failure_exhaustion_reaches_write_path();
    test_encoder_timeout_has_three_app_attempts();
    test_status_timeout_preserves_original_force_zero_policy();
    test_safety_stop_cancels_encoder_wait();
    test_unknown_read_transaction_forces_zero_without_starting_read();
    test_unknown_read_transaction_does_not_parse_completed_response();
    test_unknown_read_transaction_failure_forces_zero_without_retry();
    puts("PASS: LD2RS unified read state machine behavior");
    return 0;
}
