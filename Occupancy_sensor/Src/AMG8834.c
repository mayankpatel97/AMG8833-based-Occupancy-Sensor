#include "stm32f3xx_hal.h"
#include "var_extern.h"
#include "defs.h"

extern I2C_HandleTypeDef hi2c2;

//#define SENSOR_ADDR	0xD1    // AD_SELECT conncted to high
#define SENSOR_ADDR	0xD0	//// AD_SELECT conncted to Low

#define RESET_REG_ADRR	0x01
#define STATUS_REG_ADDR	0x04

#define INTERNAL_THERMISTOR_H_ADDR 0x0F
#define INTERNAL_THERMISTOR_L_ADDR 0x0E

#define PIXEL_0_ADDR_L	0x80
#define PIXEL_0_ADDR_H	0x81

#define PIXEL_LOWER_ADDR	128
#define PIXEL_HIGHER_ADDR	256

#define MAX_TEMP_VALUES	64
#define MAX_TEMP_ADDR		128
#define CLEAR_STATUS_REG_ADRR	0x05

#define DEF_FLAG_RESET	0x30
#define DEF_SW_RESET	0x3F

#define CLEAR_TEMPERATURE_OUTPUT_OVWERFLOW	0x04
#define CLEAR_THERMISTER_OUTPUT_OVWERFLOW	0x08
#define CLEAR_BOTH_TEMPERATURE_N_THERMISTER_OVERFLOW	0xC0


//unsigned int I2cxTimeout=0xA000;
unsigned int I2cxTimeout=500;


unsigned char Read_I2C_data(unsigned int Div_addr,unsigned char Div_reg);
void Write_I2C_data(unsigned int Div_addr, unsigned char Div_reg,unsigned char Val);
void Reset_thermal_sensor(unsigned char reset);
void Clear_staus_register(unsigned char clear_status);

void Init_thermal_sensor(void)
{
	Reset_thermal_sensor(DEF_FLAG_RESET);
	Write_I2C_data(SENSOR_ADDR,0x00,0x00); // Set power control register to normal mode
	Write_I2C_data(SENSOR_ADDR,0x02,0x00); // Set Frame rate to 10FPS
	Write_I2C_data(SENSOR_ADDR,0x03,0x00); // Disable interrupt
}

void Read_thermal_sensor(void)
{
	static unsigned int c,Status;
	for(c=0;c<MAX_TEMP_ADDR;c++)
	{
		if((c%2)==0)
			Temp_buff[c/2] = Read_I2C_data(SENSOR_ADDR,c+128);
		else
			Temp_buff[c/2] |= Read_I2C_data(SENSOR_ADDR,c+128)  << 8;
	}
	
	//----- Read Internal thermister 
	Tmp_thermistor =  Read_I2C_data(SENSOR_ADDR,INTERNAL_THERMISTOR_L_ADDR);
	Tmp_thermistor |=  (Read_I2C_data(SENSOR_ADDR,INTERNAL_THERMISTOR_H_ADDR) << 8) & 0xFF00;
	
	//---- Read status register
	Status = Read_I2C_data(SENSOR_ADDR,STATUS_REG_ADDR);
	//---- Check if overflow bits are in reset state
	if((Status & 0xC0) == 0)
	{
		//---- Temperture values are correct
		// Thermistor has 4bits of decimal value & temp_vals have 2 bits of decimal value
		
		for(c=0;c<MAX_TEMP_VALUES;c++)
			Temp_values[c] = Temp_buff[c] >> 2; // ---- Remove decimal val
		
		//----- Remove decimal value 
		Thermistor_val = Tmp_thermistor >>4;
		
	}
	else // if temperature or thermistor overflow
	{
		// Clear temperature n thermister overflow register
		Clear_staus_register(CLEAR_BOTH_TEMPERATURE_N_THERMISTER_OVERFLOW);
		
		//---- Reset sensor here
		//Reset_thermal_sensor(DEF_FLAG_RESET);
		//Reset_thermal_sensor(DEF_SW_RESET);	
	}
}


void Reset_thermal_sensor(unsigned char reset)
{
	Write_I2C_data(SENSOR_ADDR,RESET_REG_ADRR,reset);
}

void Clear_staus_register(unsigned char clear_overflow)
{
	Write_I2C_data(SENSOR_ADDR,CLEAR_STATUS_REG_ADRR,clear_overflow);
}

unsigned char Read_I2C_data(unsigned int Div_addr,unsigned char Div_reg)
{
  static unsigned char value = 0;
  HAL_I2C_Mem_Read(&hi2c2,Div_addr,Div_reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);
  return value;
}

void Write_I2C_data(unsigned int Div_addr, unsigned char Div_reg,unsigned char Val)
{
  HAL_I2C_Mem_Write(&hi2c2,Div_addr,(unsigned int)Div_reg, I2C_MEMADD_SIZE_8BIT, &Val, 1, I2cxTimeout);
}

