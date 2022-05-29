#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#include "var_extern.h"
#include "defs.h"

extern UART_HandleTypeDef huart3;

unsigned char Tx_buff3[150];

void Uart_rx3(void)
{
	if(Rx_data3 == '*')
	{
		Gui_connection_status=CONNECTED;
	}
	else if(Rx_data3 == '~')
	{
		Gui_connection_status=DISCONNECTED;
	}
}



void Send_data_2_gui(void)
{
	static unsigned char c,checksum;
	
	checksum =0;
	Tx_buff3[0] = '*';
	Tx_buff3[1] = '*';
	Tx_buff3[2] = '*';
	
	Tx_buff3[3] = Tmp_thermistor & 0x00FF;
	checksum += Tx_buff3[3];
	Tx_buff3[4] = (Tmp_thermistor & 0xFF00) >> 8;
	checksum += Tx_buff3[4];
	
	for(c=0;c<128;c++)
	{
		if((c%2)==0)
			Tx_buff3[c+5] = Temp_buff[c/2] & 0x00FF;
		else
			Tx_buff3[c+5] = (Temp_buff[c/2] & 0xFF00) >> 8;
		
			checksum += Tx_buff3[c+5];
		
	}
	
	Tx_buff3[133] = checksum;
	HAL_UART_Transmit(&huart3, &Tx_buff3[0],134, 500);
	
}