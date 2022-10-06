#include "sspi.h"
#include "stdlib.h"
#include "string.h"
extern UART_HandleTypeDef huart1;
//inline void sspi_w(uint8_t dat) {
//  uint8_t sspi_i;
//  for (sspi_i = 0x80; sspi_i != 0x00; sspi_i >>= 1) {
//	  HAL_GPIO_WritePin(SPI_SCK_GPIO_Port, SPI_SCK_Pin, 0);
//    if (dat & sspi_i) {
//    	HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_SCK_Pin, 1);
//    }
//    else {
//    	HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, 0);
//    }
//    HAL_GPIO_WritePin(SPI_SCK_GPIO_Port, SPI_SCK_Pin, 1);
//  }
//}

inline uint8_t sspi_r(void)
{
  uint8_t rdata=0;
  uint8_t i,j;
  for(i=0;i<8;i++)
  {
    HAL_GPIO_WritePin(SPI_SCK_GPIO_Port, SPI_SCK_Pin, GPIO_PIN_RESET);
    rdata <<= 1;
    if(HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin))
    {
      rdata |= 0x01;
    }
    else
    for(j=0;j<10;j++);
    HAL_GPIO_WritePin(SPI_SCK_GPIO_Port, SPI_SCK_Pin, GPIO_PIN_SET);

  }
  return rdata;
}

inline void sspi_receive(uint8_t*data, uint8_t size)
{
	uint8_t i;
	for(i=0;i<size;i++)
	{
		data[i] = sspi_r();
	}
}
