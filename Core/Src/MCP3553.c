#include "MCP3553.h"
#include "sspi.h"

#define MCP3553_DATA_LEN 3
#define MCP3553_TIMEOUT_MS 80

#define MCP3553_CS_PORT GPIOB
#define MCP3553_CS_PIN GPIO_PIN_14

#define MCP3553_SDO_RDY_PORT GPIOB
#define MCP3553_SDO_RDY_PIN GPIO_PIN_15

#define MCP3553_OVF_BIT_MASK 0xC00000
#define MCP3553_SIGN_BIT_MASK 0x200000
#define MCP3553_DATA_BIT_MASK 0x3FFFFF

#define MCP3553_VREF_V 5
#define MCP3553_DATA_MAX 0x200000
#define MCP3553_LSB_V MCP3553_VREF_V / MCP3553_DATA_MAX
extern UART_HandleTypeDef huart1;

float readMCP3553(void)
{
	uint8_t mcp3553_data[MCP3553_DATA_LEN] ;
	float mcp3553_voltage = 0;
	int32_t mcp3553_lsb = 0;
	uint32_t mcp3553_timeout_cnt = HAL_GetTick();

	HAL_GPIO_WritePin(MCP3553_CS_PORT, MCP3553_CS_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);

	while((HAL_GPIO_ReadPin(MCP3553_SDO_RDY_PORT, MCP3553_SDO_RDY_PIN) == GPIO_PIN_SET) &&
	      (HAL_GetTick()-mcp3553_timeout_cnt < MCP3553_TIMEOUT_MS));

	if (HAL_GetTick()-mcp3553_timeout_cnt < MCP3553_TIMEOUT_MS)
	{
	  sspi_receive(mcp3553_data,MCP3553_DATA_LEN);
	  uint32_t result = (mcp3553_data[0] << 16) | (mcp3553_data[1] << 8) | (mcp3553_data[2]);

	  if(!(result & MCP3553_OVF_BIT_MASK))
	  {
	    if (!(result & MCP3553_SIGN_BIT_MASK))
	    {
	      mcp3553_lsb = result;
	    }
	    else
	    {
	      mcp3553_lsb = (~result);
	      mcp3553_lsb &= MCP3553_DATA_BIT_MASK;
	      mcp3553_lsb += 1;
	      mcp3553_lsb *= (-1);
	    }
	    mcp3553_voltage = (float)mcp3553_lsb * MCP3553_LSB_V;
	  }
	}
	HAL_GPIO_WritePin(MCP3553_CS_PORT, MCP3553_CS_PIN, GPIO_PIN_SET);
	return mcp3553_voltage;
}

