#include "BMI088Middleware.h"
#include "spi.h"
#include "delay.h"
//#include "main.h"



//extern SPI_HandleTypeDef hspi1;
#define BMI08X_TIMEOUT_CNT 1680000

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
	 delay_ms(ms);
}

void BMI088_delay_us(uint16_t us)
{
   delay_us(us);
}





void BMI088_ACCEL_NS_L(void)
{
    //HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
		GPIO_ResetBits(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin);
}
void BMI088_ACCEL_NS_H(void)
{
    //HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
		GPIO_SetBits(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin);
}

void BMI088_GYRO_NS_L(void)
{
    //HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
	  GPIO_ResetBits(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin);
}
void BMI088_GYRO_NS_H(void)
{
    //HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
	  GPIO_SetBits(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
		uint32_t timeout_cnt = 0;
		//SET_BIT(SPI1->CR1, SPI_CR1_SPE);
		while((SPI1->SR & SPI_SR_TXE) == RESET)
		{
			if(timeout_cnt < BMI08X_TIMEOUT_CNT)
			{
				timeout_cnt++;
			}else return 0;
		}
		SPI1->DR = txdata;
		timeout_cnt = 0;
		while((SPI1->SR & SPI_SR_RXNE) == RESET)
		{
			if(timeout_cnt < BMI08X_TIMEOUT_CNT)
			{
				timeout_cnt++;
			}else return 0;
		}
		return SPI1->DR;
}

