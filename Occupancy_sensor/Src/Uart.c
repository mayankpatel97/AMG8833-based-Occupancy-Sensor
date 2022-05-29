#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#include "var_extern.h"
#include "defs.h"

extern UART_HandleTypeDef huart4;
void Reset_serial_data(void);

unsigned char Rx_cmd,Bcc;
//void Send_sensor_vals(void);

void Send_handshake_packet(void);
void Send_amg8833_array_64byte(void);
void Send_amg8833_array_127byte(void);

void Uart_rx(void)
{
	if(Rx_len == 0)
	{
		if(Rx_data == HEADER)
		{
			Rx_len++;
			Rx_timeout=DEF_RX_TIMEOUT;
			Bcc = HEADER;
		}
	}
	else if(Rx_len == 1)
	{
		if(Rx_data < MAX_PACKET_LENGTH)
		{
			Rx_packet_len = Rx_data;
			Bcc ^= Rx_data;
			Rx_len++;
		}
		else
		{
			Reset_serial_data();
		}
	}
	else if((Rx_len >= 2) && (Rx_len < (Rx_packet_len-1)))
	{
		Rx_buff[Rx_len-2] = Rx_data;
		Rx_len++;
		Bcc ^= Rx_data;
		Rx_timeout=DEF_RX_TIMEOUT;
	}
	else if(Rx_len == (Rx_packet_len-1))
	{
		if(Bcc == Rx_data)
		{
			Rx_cmd = Rx_buff[0];
			Rx_payload[0] = Rx_buff[1];
			//Rx_payload[1] = Rx_buff[2];			
			New_serial_data=1;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
			Rx_led_timeout = DEF_RX_LED_TIMEOUT;
			
		}
		Reset_serial_data();
	}
	else
	{
		Reset_serial_data();
	}
}


void Reset_serial_data(void)
{
	Rx_len=0;
	Rx_packet_len=0;
	Bcc=0;
	Rx_timeout=0;
}

void Analyse_serial_data(void)
{
	if(Rx_cmd == CMD_QUERY)
	{
		//Send_sensor_vals();
		/*
		if(Rx_payload[0])
			Send_amg8833_array_127byte();
		else
			Send_amg8833_array_64byte();
		*/
		Send_amg8833_array_64byte();
		//Send_amg8833_array_127byte();
		
	}
	else if(Rx_cmd == CMD_HANDSHAKE)
	{
		Send_handshake_packet();
	}
	
	Reset_serial_data();
}

/*
void Send_sensor_vals(void)
{
	static unsigned char c;
	
	Tx_buff[0] = HEADER;
	Tx_bcc = HEADER;
	
	Tx_buff[1] = DEF_SENSOR_VAL_PACKET_LEN;
	Tx_bcc ^= DEF_SENSOR_VAL_PACKET_LEN;
	
	Tx_buff[2] = CMD_SEND_SENSOR_VAL;		
	Tx_bcc ^= CMD_SEND_SENSOR_VAL;
	
	for(c=0;c<34;c++)
	{
		Tx_buff[c+3] = Th_sensor_buff[c];
		Tx_bcc ^= Th_sensor_buff[c];
	}
	
	Tx_buff[37] =	Tx_bcc;
	
	HAL_UART_Transmit(&huart4, &Tx_buff[0], DEF_SENSOR_VAL_PACKET_LEN, 500);
}
*/

void Send_amg8833_array_64byte(void)
{
	static unsigned char c;

	Tx_buff[0] = HEADER;
	Tx_bcc = HEADER;
	
	Tx_buff[1] = DEF_AMG8834_DATA_64_PACKET_LEN;
	Tx_bcc ^= DEF_AMG8834_DATA_64_PACKET_LEN;
	
	Tx_buff[2] = CMD_SEND_AMG8834_VAL;		
	Tx_bcc ^= CMD_SEND_AMG8834_VAL;
	
	for(c=0;c<64;c++)
	{
		Tx_buff[c+3] = Temp_values[c] & 0x00FF;
		Tx_bcc	^= Tx_buff[c+3];
	}
	
	Tx_buff[67] = Thermistor_val & 0x00FF;
	Tx_bcc ^= Tx_buff[67];
	
	Tx_buff[68] = Tx_bcc;
	
	HAL_UART_Transmit(&huart4, &Tx_buff[0],DEF_AMG8834_DATA_64_PACKET_LEN, 500);
}


void Send_amg8833_array_127byte(void)
{
	static unsigned int c;
	
	Tx_buff[0] = HEADER;
	Tx_bcc = HEADER;
	
	Tx_buff[1] = DEF_AMG8834_DATA_127_PACKET_LEN;
	Tx_bcc ^= DEF_AMG8834_DATA_127_PACKET_LEN;
	
	Tx_buff[2] = CMD_SEND_AMG8834_VAL;		
	Tx_bcc ^= CMD_SEND_AMG8834_VAL;
	
	// 1/2 to be tested
	for(c=0;c<128;c++)
	{
		if((c%2)==0)
			Tx_buff[c+3] = Temp_values[c/2] & 0x00FF;
		else
			Tx_buff[c+3] = (Temp_values[c/2] & 0xFF00) >> 8;
		
		Tx_bcc ^= Tx_buff[c+3];
	}
	
	Tx_buff[131] = Thermistor_val & 0x00FF;
	Tx_bcc ^= Tx_buff[131];
	Tx_buff[132] = (Thermistor_val & 0xFF00) >> 8;
	Tx_bcc ^= Tx_buff[132];
	
	Tx_buff[133] = Tx_bcc;
	
	HAL_UART_Transmit(&huart4, &Tx_buff[0],DEF_AMG8834_DATA_127_PACKET_LEN, 500);
	
	
}

void Send_handshake_packet(void)
{
	Tx_buff[0] = HEADER;
	Tx_bcc = HEADER;
	
	Tx_buff[1] = DEF_HANDSHAKE_PACKET_LEN;
	Tx_bcc ^= DEF_HANDSHAKE_PACKET_LEN;
	
	Tx_buff[2] = CMD_HANDSHAKE;		
	Tx_bcc ^= CMD_HANDSHAKE;
	
	Tx_buff[3] = 0x01;		
	Tx_bcc ^= 0x01;
	
	Tx_buff[4] = Tx_bcc;
	HAL_UART_Transmit(&huart4, &Tx_buff[0],DEF_HANDSHAKE_PACKET_LEN, 500);
}

