#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

/* Private defines -----------------------------------------------------------*/
#define sensor_right_Pin GPIO_Pin_7
#define sensor_right_GPIO_Port GPIOI
#define sensor_left_Pin GPIO_Pin_6
#define sensor_left_GPIO_Port GPIOI
#define LED_R_Pin GPIO_Pin_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_Pin_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_Pin_10
#define LED_B_GPIO_Port GPIOH

#define CS1_ACCEL_Pin GPIO_Pin_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_Pin_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_Pin_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_Pin_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif
