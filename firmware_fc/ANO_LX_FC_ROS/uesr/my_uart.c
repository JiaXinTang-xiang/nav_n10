#include "my_uart.h"
#include "Drv_Uart.h"
#include "AnoDTRasp.h"
#include "mode.h"
#include "get_data.h"


unsigned char  data1[]={0x23,0x33,0x40};	//返回k230，停止扫描  #3@
u8 length=3;

void Send_Data_To_32(u8* send_data, u8 data_len);

static uint8_t data_temp;
int K230_Flag = 0;
static u8 tx_buffer[20];


void Recive_K230data(uint8_t data)////接收k230的数据
{
	static uint8_t RxState = 0;	//定义表示当前状态机状态的静态变量
	
	if(RxState == 0 && data == 0x23)	//如果第一个数据是帧头#
	{
		RxState = 1;
	}
	else if(RxState == 1 && (data == 0x31||data == 0x32))	//如果第二个数据是 1 或者 4
	{
		RxState = 2;
		data_temp = data;
	}
	else if(RxState == 2 && data == 0x40)	//如果第三个数据是帧尾@
	{
		K230_Flag = data_temp;	//此时则可以将数据赋给K230_Flag
		data_temp = 0;			//将暂缓区清零
		RxState = 0;			//重置标志位
		
	}
	else
	{
		RxState = 0;
		data_temp = 0;
		K230_Flag = 0;
	}
}

///*********飞控发送函数************************/

void Send_Data_To_32(u8* send_data, u8 data_len)
{
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0xF5;
	tx_buffer[2] = data_len;
	
	for(u8 i = 0; i < data_len; i++)
	{
		tx_buffer[3+i] = send_data[i];
	}
	
	u8 check_sum1 = 0, check_sum2 = 0;
	for(u8 j = 0; j < data_len + 3; j++)
	{
		check_sum1 += tx_buffer[j];
		check_sum2 += check_sum1;
	}
	
	tx_buffer[data_len+3] = check_sum1;
	tx_buffer[data_len+4] = check_sum2;
	// 实际发送
	DrvUart1SendBuf(tx_buffer, data_len+5);
}
