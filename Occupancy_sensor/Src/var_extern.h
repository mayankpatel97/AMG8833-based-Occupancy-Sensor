//------------UART
extern unsigned char Rx_data,Uart_no,Rx_result;
extern _Bool New_serial_data;
extern unsigned char Rx_buff[50],Rx_data,Rx_bcc,Rx_packet_len,Rx_payload[20];
extern unsigned int Rx_len,Rx_timeout;
extern unsigned char Tx_buff[150],Tx_bcc;

extern unsigned char Rx_data3,Gui_connection_status;
//------------Timer
extern unsigned int Small_ticks,Ticks,Secs;
extern _Bool Small_tick_over,One_tick_over,Ms50_over,Ms100_over,Half_sec_over,One_sec_over,Mini_tick_over;
extern _Bool Sec20_over,One_min_over;
extern unsigned int Rx_led_timeout;

//------------ Sensor
extern const unsigned int Write_add;
extern const unsigned int Read_add;

extern unsigned char Th_cmd[10],I2C_read_buff[50],Th_sensor_buff[50];
extern unsigned int Th_ptat;
extern int Ptat,Pec,Thermal_sensor_val[20],Sensor_data_correct;

//-------------  AMG8834

extern unsigned int Temp_buff[150],Thermistor_val,Tmp_thermistor,Temp_values[150];
extern unsigned char Sensor_val_buff[64],Amg_temp_buff[150];
