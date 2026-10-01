#include "mycan.h"

void MyCAN_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	
	
	
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	
	CAN_InitTypeDef caninitstructure;
	//CAN_Mode_LoopBack
    caninitstructure.CAN_Mode=CAN_Mode_Normal;//设置can外设的测试模式
    caninitstructure.CAN_Prescaler= 4;//分频系数     //波特率=APB1时钟频率/分频系数/一位的TQ的量=36MHz/(BRP[9:0+1])/(1+(TS1[3:0]+1))+(TS2[2:0]+1))
    caninitstructure.CAN_BS1=CAN_BS1_9tq;//BS1        //波特率=36M/48/(1=2=3)=125k
    caninitstructure.CAN_BS2=CAN_BS2_8tq;//BS2
    caninitstructure.CAN_SJW=CAN_SJW_1tq;
	//位时序
    caninitstructure.CAN_NART=DISABLE;//NART置1关闭自动重传，置0会自动重传直到成功
    caninitstructure.CAN_TXFP=DISABLE;//TXFP置1优先级由发送请求的顺序来决定，先请求的先发送；置0优先级由报文标识符来决定，小的先发送
    caninitstructure.CAN_RFLM=DISABLE;//RFLM置1接收FIFO锁定，FIFO溢出时新收到的报文会被丢弃，置0时则会覆盖最后的报文
	caninitstructure.CAN_AWUM=DISABLE;//置1自动唤醒，一旦检测到can总线活动，硬件就自动清零SLEEP，唤醒CAN外设；置0手动唤醒，软件清零SLEEP唤醒CCAN外设
    caninitstructure.CAN_TTCM=DISABLE;//置1开启时间触发通信功能；置零关闭时间触发通信功能
	caninitstructure.CAN_ABOM=DISABLE;//置1开启离线自动恢复，进入离线状态后，就自动开启恢复过程；置0，关闭离线自动恢复，软件必须先请求进入然后再退出初始化模式，随后恢复过程才被开启
	CAN_Init(CAN1,&caninitstructure);
	
	
	CAN_FilterInitTypeDef canfilterinitstucture;
	canfilterinitstucture.CAN_FilterNumber=0;//指定初始化0到13的过滤器
	canfilterinitstucture.CAN_FilterIdHigh=0x0000;
	canfilterinitstucture.CAN_FilterIdLow=0x0000;
	canfilterinitstucture.CAN_FilterMaskIdHigh=0x0000;//屏蔽器全给0代表全通
	canfilterinitstucture.CAN_FilterMaskIdLow=0x0000;
	
	canfilterinitstucture.CAN_FilterScale=CAN_FilterScale_32bit;//指定过滤器位宽1
	canfilterinitstucture.CAN_FilterMode=CAN_FilterMode_IdMask;//选择过滤器模式为屏蔽模式
	canfilterinitstucture.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//配置过滤器关联
	canfilterinitstucture.CAN_FilterActivation=ENABLE;//激活过滤器以让报文通过
	
	
	
	CAN_FilterInit(&canfilterinitstucture);//函数内部会把FINIT置1，再失能过滤器，就可以进行过滤器的初始化了
	
	
}

void MyCAN_Transmit(CanTxMsg *TxMessage )//发送部分
{
  
	uint8_t TransmitMailbox = CAN_Transmit(CAN1,TxMessage);//返回值为选中的邮箱
	
	uint32_t Timeout=0;
    while(CAN_TransmitStatus(CAN1,TransmitMailbox)!=CAN_TxStatus_Ok)
    {
       Timeout ++;
		if(Timeout>100000)
		{
		   break;
		}
 
    }
}//发送部分


void MyCAN_Transmit_Double(CanTxMsg *TxMessage , CanTxMsg *TxMessage2)//发送部分
{
  
	uint8_t TransmitMailbox = CAN_Transmit(CAN1,TxMessage);//返回值为选中的邮箱
	uint8_t TransmitMailbox2 = CAN_Transmit(CAN1,TxMessage2);
	uint32_t Timeout=0;
    while(CAN_TransmitStatus(CAN1,TransmitMailbox)!=CAN_TxStatus_Ok)
    {
       Timeout ++;
		if(Timeout>100000)
		{
		   break;
		}
 
    }
}//发送部分
uint8_t MyCAN_ReceiveFlag(void)
{
 if(CAN_MessagePending(CAN1,CAN_FIFO0)>0)//检测fifo 0队列的队伍长度
 {
 return 1;
 }
 return 0;
}

void MyCAN_Receive(CanRxMsg *RxMessage)
{
   
   CAN_Receive(CAN1,CAN_FIFO0,RxMessage);	

}

