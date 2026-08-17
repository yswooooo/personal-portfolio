/**
  ******************************************************************************
  * @file    app_encoder_dev.h
  * @brief   编码器位置采样、角度增量和速度估算接口
  ******************************************************************************
  */

#ifndef APP_ENCODER_DEV_H
#define APP_ENCODER_DEV_H

#include <stdint.h>

/**
  * @brief 一次有效的电机侧编码器位置采样。
  * @note  发布者写完 counts 和 timestamp_us 后递增 sequence，消费者据此判断是否有新样本。
  */
typedef struct
{
    int32_t counts;       /**< PrB.24 电机侧累计位置反馈，count，有符号 32 bit。 */
    uint64_t timestamp_us; /**< 接收并解析该反馈时的 DWT 时间戳，us。 */
    uint32_t sequence;    /**< 有效样本发布序号，按 uint32_t 自然回绕。 */
} app_encoder_sample_t;

/**
  * @brief 基于相邻两次有效编码器采样的速度与角度差分结果。
  * @note  initialized 表示已有历史样本；speed_valid 表示本次差分结果可用。
  */
typedef struct
{
    uint8_t initialized;             /**< 历史采样初始化标志：0=无历史样本，1=已保存。 */
    uint8_t speed_valid;             /**< 当前估算结果有效标志：0=无效，1=有效。 */
    uint8_t is_stale;                /**< 估算结果陈旧标志：0=新鲜，1=超时未更新。 */

    int32_t last_counts;             /**< 上一次有效编码器累计位置，count。 */
    uint64_t last_timestamp_us;      /**< 上一次有效样本的 DWT 时间戳，us。 */
    uint32_t last_sequence;          /**< 已处理的最近样本序号。 */

    int64_t delta_counts;            /**< 极性修正后的相邻编码器位置差，count。 */
    uint64_t delta_time_us;          /**< 相邻有效样本的时间差，us。 */
    float delta_motor_angle_rad;     /**< 电机转子在本采样间隔内的增量角度，rad。 */
    float delta_wheel_angle_rad;     /**< 经过减速比后的车轮增量角度，rad。 */
    float speed_counts_per_s;        /**< 电机侧编码器变化率，count/s。 */
    float motor_speed_rpm;           /**< 电机转子估算转速，rpm。 */
    float wheel_speed_rpm;           /**< 经过减速比后的车轮估算转速，rpm。 */
    float wheel_speed_mps;           /**< 由车轮半径换算得到的线速度，m/s。 */
} app_encoder_estimator_t;

/**
  * @brief 使用新编码器样本更新角度增量和速度估算结果。
  * @param[in]     sample           已发布的编码器位置样本。
  * @param[in,out] estimator        速度估算器及其历史采样状态。
  * @param[in]     encoder_polarity 编码器方向极性，只允许使用 1 或 -1。
  * @note  sample 或 estimator 为 NULL、sequence 未变化时不更新；首个样本只建立历史基准。
  */
void app_encoder_update(
    const app_encoder_sample_t *sample,
    app_encoder_estimator_t *estimator,
    int8_t encoder_polarity);

#endif /* APP_ENCODER_DEV_H */
