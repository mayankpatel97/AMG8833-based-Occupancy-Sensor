//------------UART
unsigned char Rx_data,Uart_no,Rx_result;
_Bool New_serial_data=0;
unsigned char Rx_buff[50],Rx_data=0,Rx_bcc=0,Rx_packet_len=0,Rx_payload[20];
unsigned int Rx_len=0,Rx_timeout=0;
unsigned char Tx_buff[150],Tx_bcc =0;

unsigned char Rx_data3,Gui_connection_status;
//------------Timer
unsigned int Small_ticks=0,Ticks=0,Secs=0;
_Bool Small_tick_over=0,One_tick_over=0,Ms50_over=0,Ms100_over=0,Half_sec_over=0,One_sec_over=0,Mini_tick_over=0;
_Bool Sec20_over=0,One_min_over=0;

unsigned int Rx_led_timeout=0;

//------------ Sensor

const unsigned int Write_add = 0x14;
const unsigned int Read_add = 0x15;

unsigned char Th_cmd[10],I2C_read_buff[50],Th_sensor_buff[50];
unsigned int Th_ptat;
int Ptat,Pec,Thermal_sensor_val[20],Sensor_data_correct;

//-------------  AMG8834

unsigned int Temp_buff[150],Thermistor_val,Tmp_thermistor,Temp_values[150];
unsigned char Sensor_val_buff[64],Amg_temp_buff[150];
