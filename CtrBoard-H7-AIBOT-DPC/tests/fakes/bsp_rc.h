#ifndef TEST_BSP_RC_H
#define TEST_BSP_RC_H

#include <stdint.h>

enum {
    eRC_POS_DOWN = -800,
    eRC_POS_MID = 0,
    eRC_POS_UP = 800
};

enum {
    eRC_SW_A = 0,
    eRC_SW_B = 1,
    eRC_SW_C = 2,
    eRC_SW_D = 3
};

typedef struct {
    int16_t curr;
    int16_t prev;
} RC_SwitchState_t;

typedef struct {
    int16_t ch_ry;
    int16_t ch_rx;
    int16_t ch_lx;
    int16_t ch_ly;
    int16_t sw_val[4];
    RC_SwitchState_t sw_st[4];
    int16_t vra;
    int16_t vrb;
    int16_t _pad[5];
    uint8_t lost_flag;
} RC_Channels_t;

typedef struct {
    float ch_ry;
    float ch_rx;
    float ch_lx;
    float ch_ly;
} RC_Filter_t;

typedef struct {
    float fLinearVel;
    float fAngularVel;
    int16_t i16LeftRpm;
    int16_t i16RightRpm;
} RC_ChassisCmd_t;

extern RC_Channels_t g_rc;
extern RC_Filter_t g_rc_filter;
extern RC_ChassisCmd_t g_rc_chassis;

#endif
