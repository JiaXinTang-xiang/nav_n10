#include "AnoDTRasp.h"
#include "Drv_Uart.h"
#include "ANO_DT_LX.h"
#include "Drv_led.h"
#include "LX_FC_EXT_Sensor.h"
#include "ANO_LX.h"

static void anoDTRaspDataFrameAnl(void);
static void anoDTRaspSendCheck(void);
static void Ano_FrameID_Send(u8 frame_id);

//接收数据缓冲
static myTAnoPTv7Frame rxFrame;
//发送数据缓冲
#define ANOPTV8TXBUFNUM	20
static myTPTv7TxFrame txFrameBuf[ANOPTV8TXBUFNUM];
static u8 txBufWAddr = 0;
static u8 txBufRAddr = 0;

ROS_DATA_ST rosData;//接受数据结构体

//
void AnoDTRaspRunTask1Ms(void)
{
	static u16 tmp_cnt[2];
	//计5ms
	tmp_cnt[0]++;
	tmp_cnt[0] %= 2;
	if (tmp_cnt[0] == 0)
	{
        Ano_FrameID_Send(0x07);//飞行速度数据
        Ano_FrameID_Send(0x02);//罗盘数据
        Ano_FrameID_Send(0x40);//遥控器数据
		//计10ms
		tmp_cnt[1]++;
		tmp_cnt[1] %= 2;
		if (tmp_cnt[1] == 0)
		{
			Ano_FrameID_Send(0x30);//gps数据
			AnoPTv7SendUserData(10,2,100,-303,303,3333);//用户数据
			Ano_FrameID_Send(0x05);//高度数据
		}
	}
    Ano_FrameID_Send(0x01);//惯性传感器数据
    Ano_FrameID_Send(0x04);//四元素数据
	//检查是否有数据需要发送
	anoDTRaspSendCheck();
}
//
void AnoDTRaspRecvOneByte(u8 data)
{
	static u16 _recv_cnt = 0;
	static u16 _data_cnt = 0;
	static u8 rxstate = 0;

	//判断帧头是否满足匿名协议的0xAB
	if (rxstate == 0 && data == 0xAA)
	{
		_recv_cnt = 0;
		rxFrame.rawBytes[_recv_cnt++] = data;
		rxstate ++;
	}
	//数据地址字节
	else if (rxstate == 1)
	{
		rxFrame.rawBytes[_recv_cnt++] = data;
		rxstate ++;
	}
	//帧ID
	else if (rxstate == 2)
	{
		rxFrame.rawBytes[_recv_cnt++] = data;
		rxstate ++;
	}
	//接收数据长度
	else if (rxstate == 3)
	{
		rxFrame.rawBytes[_recv_cnt++] = data;
		rxFrame.frame.dataLen = data;
		_data_cnt = 0;
		rxstate ++;
	}
	//接收数据
	else if (rxstate == 4)
	{
		rxFrame.rawBytes[_recv_cnt++] = data;
		_data_cnt++;
		if(_data_cnt >= rxFrame.frame.dataLen)
		{
			rxstate ++;
		}
	}
	//接收SC1
	else  if (rxstate == 5)
	{
		rxFrame.frame.sc1 = data;
		rxstate ++;
	}
	//接收SC2
	else  if (rxstate == 6)
	{
		rxFrame.frame.sc2 = data;
		rxstate = 0;
		//所有数据接收完毕，进行解析
		anoDTRaspDataFrameAnl();
	}
	else
	{
		rxstate = 0;
	}
}


static void anoDTRaspDataFrameAnl(void)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	//根据收到的数据计算校验字节1和2
	for (u16 i = 0; i < (rxFrame.frame.dataLen + 4); i++)
	{
		check_sum1 += rxFrame.rawBytes[i];
		check_sum2 += check_sum1;
	}
	//计算出的校验字节和收到的校验字节做对比，完全一致代表本帧数据合法，不一致则跳出解析函数
	if ((check_sum1 != rxFrame.frame.sc1) || (check_sum2 != rxFrame.frame.sc2)) //判断sum校验
		return;
    /*******************************************/
    switch(rxFrame.frame.ID)
    {
		case 0x31:
		{			
			rosData.acc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
			rosData.acc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
			rosData.acc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
			rosData.yaw    = (s16)*((s16*)(rxFrame.frame.dataBuf + 6));
		}
		break;
        case 0x32:
		{			
			rosData.tacc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
			rosData.tacc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
			rosData.tacc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
			
			rosData.tloc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 6));
			rosData.tloc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 8));
			rosData.tloc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 10));
		}
		break;
		case 0x33:
		{
			rosData.loc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
			rosData.loc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
			rosData.loc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
		}break;
		default:
			break;
	}
}


//
void AnoPTv7DrvSend(u8 linktype, u8 *buf, u16 len)
{
	DrvUart3SendBuf(buf,len);
}
//
//将需要发送的帧写入缓冲区
static void anoPTv7AddTxFrame(u8 linkType, myTAnoPTv7Frame *pFrame)
{
	if(txFrameBuf[txBufWAddr].cmd._isUsed)
		return;
	txFrameBuf[txBufWAddr].cmd._isUsed = 1;
	txFrameBuf[txBufWAddr].cmd._linkType = linkType;
	for(u16 i = 0; i < (pFrame->frame.dataLen + 6); i++)
		txFrameBuf[txBufWAddr].data.rawBytes[i] = pFrame->rawBytes[i];
	
	txFrameBuf[txBufWAddr].cmd._isUsed = 2;
	
	txBufWAddr++;
	if(txBufWAddr >= ANOPTV8TXBUFNUM)
		txBufWAddr = 0;
}
//
static void anoDTRaspSendCheck(void)
{
	//如果发送缓冲区有需要发送的数据，执行发送
	for(u8 i=0; i<ANOPTV8TXBUFNUM; i++)
	{
		if(txFrameBuf[txBufRAddr].cmd._isUsed == 2)
		{
			//数据写入完毕，调用发送
			AnoPTv7DrvSend(txFrameBuf[txBufRAddr].cmd._linkType, txFrameBuf[txBufRAddr].data.rawBytes, txFrameBuf[txBufRAddr].data.frame.dataLen+6);
			txFrameBuf[txBufRAddr].cmd._isUsed = 0;
		}
		txBufRAddr++;
		if(txBufRAddr >= ANOPTV8TXBUFNUM)
		txBufRAddr = 0;
	}
}

//发送用户数据
void AnoPTv7SendUserData(u8 d1,u8 d2,u8 d3,s16 d4,s16 d5,s16 d6)
{
	myTAnoPTv7Frame txFrame;
	txFrame.frame.head = 0xAA;
	txFrame.frame.addr = 0xFF;
	txFrame.frame.ID = 0xF1;
	txFrame.frame.dataLen = 9;
	txFrame.frame.dataBuf[0] = d1;
	txFrame.frame.dataBuf[1] = d2;
	txFrame.frame.dataBuf[2] = d3;

	txFrame.frame.dataBuf[3] = BYTE0(d4);
	txFrame.frame.dataBuf[4] = BYTE1(d4);

	txFrame.frame.dataBuf[5] = BYTE0(d5);
	txFrame.frame.dataBuf[6] = BYTE1(d5);

	txFrame.frame.dataBuf[7] = BYTE0(d6);
	txFrame.frame.dataBuf[8] = BYTE1(d6);
	
	u8 sc1 = 0;
	u8 sc2 = 0;
	for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
	{
		sc1 += txFrame.rawBytes[i];
		sc2 += sc1;
	}
	txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
	txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
	anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}


static void Add_SendData(u8 frame_id, u8 *_cnt, u8 send_buffer[])
{
	//根据需要发送的帧ID，也就是frame_id，来填充数据，填充到send_buffer数组内
	switch (frame_id)
	{
        case 0x01: //惯性传感器数据
        {
            for(u8 i=0;i<12;i++) //不发送状态标准位
            { 
                send_buffer[(*_cnt)++] = fc_acc.byte_data[i];
            }
        }break;
        case 0x02://罗盘数据
        {
            for(u8 i=0;i<6;i++)//只发送罗盘数据
            { 
                send_buffer[(*_cnt)++] = fc_mag.byte_data[i];
            }
        }break;
        case 0x04://四元素数据
        {
            for(u8 i=0;i<8;i++) //不发送状态标准位
            {
                send_buffer[(*_cnt)++] = fc_att_qua.byte_data[i];
            }	
        }break;
        case 0x05://高度数据
        {
            for(u8 i=0;i<8;i++) //不发送状态标准位
            {
                send_buffer[(*_cnt)++] = fc_alt.byte_data[i];
            }	
        }break;
        case 0x07://飞行速度数据
        {
            for(u8 i=0;i<6;i++)
            {
                send_buffer[(*_cnt)++] = fc_vel.byte_data[i];
            }
        }break;
        case 0x0d: //电池数据
        {
            for (u8 i = 0; i < 4; i++)
            {
                send_buffer[(*_cnt)++] = fc_bat.byte_data[i];
            }
        }
        break;
        case 0x30: //GPS数据
        {
            for (u8 i = 0; i < 23; i++)
            {
                send_buffer[(*_cnt)++] = ext_sens.fc_gps.byte[i];
            }
        }
        break;
        case 0x40: //遥控数据帧
        {
            for (u8 i = 0; i < 20; i++)
            {
                send_buffer[(*_cnt)++] = rc_in.rc_ch.byte_data[i];
            }
        }
        break;
        default:
            break;
        }
}

static void Ano_FrameID_Send(u8 frame_id)
{
	u8 _cnt = 0;
	myTAnoPTv7Frame txFrame;
	txFrame.frame.head = 0xAA; 
	txFrame.frame.addr = 0xFF;
	txFrame.frame.ID = frame_id;
	//根据ID添加数据
	Add_SendData(frame_id,&_cnt,txFrame.frame.dataBuf);
	//反馈数据长度
	txFrame.frame.dataLen = _cnt;
	//计算校验
	u8 sc1 = 0;
	u8 sc2 = 0;
	for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
	{
		sc1 += txFrame.rawBytes[i];
		sc2 += sc1;
	}
    //校验添加在数组里面
	txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
	txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
	anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//发送关机指令
void AnoPTv7Send_ShutDown(void)
{
	uint8_t _buf[1] = {0x00};
	AnoPTv7SendBuf(0xFA,_buf,1);
}

//发送重启指令
void AnoPTv7Send_Restart(void)
{
	uint8_t _buf[1] = {0x00};
	AnoPTv7SendBuf(0xFB,_buf,1);
}

//通用发送函数
void AnoPTv7SendBuf(const uint8_t ID,const uint8_t* buf, const uint8_t len)
{
  
  myTAnoPTv7Frame txFrame;
  uint8_t _cnt = 0;

  txFrame.frame.head = 0xAA;
  txFrame.frame.addr = 0xFF;
  txFrame.frame.ID = ID;
  for(uint8_t i = 0; i < len; i++)
  {
    txFrame.frame.dataBuf[_cnt++] = *(buf + i);
  }
  txFrame.frame.dataLen = _cnt;
  uint8_t sc1 = 0;
  uint8_t sc2 = 0;
  for(uint16_t i=0; i<(txFrame.frame.dataLen+4); i++)
  {
    sc1 += txFrame.rawBytes[i];
    sc2 += sc1;
  }
  txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
  txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;

  anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}
