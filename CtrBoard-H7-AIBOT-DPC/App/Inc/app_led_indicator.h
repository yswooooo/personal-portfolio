/**
  ******************************************************************************
  * @file    app_led_indicator.h
  * @brief   系统状态 LED 指示 (应用层)
  ******************************************************************************
  */

#ifndef APP_LED_INDICATOR_H
#define APP_LED_INDICATOR_H

/**
  * @brief 初始化系统状态指示灯。
  * @note  系统启动阶段调用一次，初始化后 LED 显示红色。
  */
void app_led_indicator_init(void);

/**
  * @brief 根据急停、设备离线和遥控器状态刷新系统指示灯。
  * @note  在主循环中周期调用；显示优先级为急停、设备离线、待命、运行。
  */
void app_led_indicator_update(void);

#endif /* APP_LED_INDICATOR_H */


