#include "stm32f10x.h"
#include "cansteppermotor.h"
#include "mycan.h"
#include "string.h"
#include "Delay.h"
void CanMotor_SentData(u8 *PaData);
void bytes_to_float(uint8_t *buf);

u8 CRC_caculate(u8 *PaData)
{
	u16 Data_CrcSum = 0;
	for (int i = 0; i < 9; i++)
	{
		Data_CrcSum += PaData[i];
	}
	Data_CrcSum = Data_CrcSum & 0xff;
	return Data_CrcSum;
}
void CanMotor_Init()
{
	MyCAN_Init();
};
void CanMotor_CanSpeedMode_Run(u8 address, u16 Accel, fp32 onespeed, u8 adr1, u8 address2, u16 Accel2, fp32 onespeed2, u8 adr2)
{
	u16 CRC11 = 0;
	converter_t1 accel;
	converter_t2 speed;
	CanTxMsg msg1;
	CanTxMsg msg2;
	u16 CRC112 = 0;
	converter_t1 accel2;
	converter_t2 speed2;
	CanTxMsg msg3;
	CanTxMsg msg4;
	accel.Acceleration = Accel;
	speed.f_val = onespeed;
	msg1.StdId = 0x1001;
	msg1.ExtId = 0x00001001;
	msg1.IDE = CAN_Id_Extended;
	msg1.RTR = CAN_RTR_Data;
	msg1.DLC = 8;
	msg1.Data[0] = 0xC5;
	msg1.Data[1] = address;
	msg1.Data[2] = 0xF1;
	msg1.Data[3] = (~adr1) & 0x01;
	msg1.Data[4] = accel.bytes2[0];
	msg1.Data[5] = speed.bytes[3];
	msg1.Data[6] = speed.bytes[2];
	msg1.Data[7] = speed.bytes[1];

	for (int h = 0; h < 8; h++)
	{
		CRC11 += msg1.Data[h];
	}
	CRC11 += speed.bytes[0];

	msg2.StdId = 0x1001;
	msg2.ExtId = 0x00001002;
	msg2.IDE = CAN_Id_Extended;
	msg2.RTR = CAN_RTR_Data;
	msg2.DLC = 3;
	msg2.Data[0] = speed.bytes[0];
	msg2.Data[1] = CRC11;
	msg2.Data[2] = 0x5C;

	///////////////////////////////////

	accel2.Acceleration = Accel2;
	speed2.f_val = onespeed2;
	msg3.StdId = 0x1001;
	msg3.ExtId = 0x00001003;
	msg3.IDE = CAN_Id_Extended;
	msg3.RTR = CAN_RTR_Data;
	msg3.DLC = 8;
	msg3.Data[0] = 0xC5;
	msg3.Data[1] = address2;
	msg3.Data[2] = 0xF1;
	msg3.Data[3] = adr2;
	msg3.Data[4] = accel2.bytes2[0];
	msg3.Data[5] = speed2.bytes[3];
	msg3.Data[6] = speed2.bytes[2];
	msg3.Data[7] = speed2.bytes[1];

	for (int h = 0; h < 8; h++)
	{
		CRC112 += msg3.Data[h];
	}
	CRC112 += speed2.bytes[0];

	msg4.StdId = 0x1001;
	msg4.ExtId = 0x00001004;
	msg4.IDE = CAN_Id_Extended;
	msg4.RTR = CAN_RTR_Data;
	msg4.DLC = 3;
	msg4.Data[0] = speed2.bytes[0];
	msg4.Data[1] = CRC112;
	msg4.Data[2] = 0x5C;

	MyCAN_Transmit(&msg1);
	MyCAN_Transmit(&msg2);
	Delay_ms(2);
	MyCAN_Transmit(&msg3);
	MyCAN_Transmit(&msg4);
};

/*				Make by 轩
 *
 * 函数名: CanMotor_CanSpeedMode_Run
 * 描述:   通过CAN总线控制两个步进电机以位置模式运行
 * 参数:   address  - 电机1的CAN地址0x01
 *         Accel    - 电机1的加速度( 0-400 )
 *         onespeed - 电机1的目标速度( 范围: 0-2000 )
 *         position1- 电机1的目标位置( 单位: 步长，96000轮子一圈 )
 *         adr1     - 电机1的方向控制位( 0表示正转，1表示反转 )

 *         address2 - 电机2的CAN地址0x02
 *         Accel2   - 电机2的加速度( 0-400 )
 *         onespeed2- 电机2的目标速度( 范围: 0-2000 )
 *         position2- 电机2的目标位置( 单位: 步长，96000轮子一圈 )
 *         adr2     - 电机2的方向控制位( 0表示正转，1表示反转 )
 * 返回值: 无

 * 通信协议:
 *   - 电机1数据分两帧发送: 扩展ID 0x00001001(8字节) + 0x00001002(3字节)
 *   - 电机2数据分两帧发送: 扩展ID 0x00001003(8字节) + 0x00001004(3字节)
 *   - 帧头标识: 0xC5, 帧尾标识: 0x5C
 *   - 校验方式: 累加和校验(SUM CRC)
 *   - 两电机发送间隔: 2ms
 */
void CanMotor_CanPositionMode_Run(u8 address, u16 Accel, u16 onespeed1, u32 position1, u8 adr1, u8 address2, u16 Accel2, u16 onespeed2, u32 position2, u8 adr2)
{
	u16 CRC13 = 0;

	converter_t2 Position1;
	converter_t3 speed1;
	CanTxMsg msg5;
	CanTxMsg msg6;
	u16 CRC14 = 0;

	converter_t2 Position2;
	converter_t3 speed2;
	CanTxMsg msg7;
	CanTxMsg msg8;

	speed1.u16_val = onespeed1;
	Position1.u32_val = position1;
	msg5.StdId = 0x1005;
	msg5.ExtId = 0x00001005;
	msg5.IDE = CAN_Id_Extended;
	msg5.RTR = CAN_RTR_Data;
	msg5.DLC = 8;
	msg5.Data[0] = 0xC5;
	msg5.Data[1] = address;
	msg5.Data[2] = 0xF3;
	msg5.Data[3] = (~adr1) & 0x01;
	msg5.Data[4] = Accel;
	msg5.Data[5] = speed1.bytes3[1];
	msg5.Data[6] = speed1.bytes3[0];
	msg5.Data[7] = Position1.bytes[3];
	for (int h = 0; h < 8; h++)
	{
		CRC13 += msg5.Data[h];
	}
	for (int h = 0; h < 3; h++)
	{
		CRC13 += Position1.bytes[h];
	}

	msg6.StdId = 0x1001;
	msg6.ExtId = 0x00001006;
	msg6.IDE = CAN_Id_Extended;
	msg6.RTR = CAN_RTR_Data;
	msg6.DLC = 5;
	msg6.Data[0] = Position1.bytes[2];
	msg6.Data[1] = Position1.bytes[1];
	msg6.Data[2] = Position1.bytes[0];
	msg6.Data[3] = CRC13 & 0xff;
	msg6.Data[4] = 0x5C;

	///////////////////////////////////
	speed2.u16_val = onespeed2;
	Position2.u32_val = position2;
	msg7.StdId = 0x1001;
	msg7.ExtId = 0x00001007;
	msg7.IDE = CAN_Id_Extended;
	msg7.RTR = CAN_RTR_Data;
	msg7.DLC = 8;
	msg7.Data[0] = 0xC5;
	msg7.Data[1] = address2;
	msg7.Data[2] = 0xF3;
	msg7.Data[3] = adr2;
	msg7.Data[4] = Accel2;
	msg7.Data[5] = speed2.bytes3[1];
	msg7.Data[6] = speed2.bytes3[0];
	msg7.Data[7] = Position2.bytes[3];
	for (int h = 0; h < 8; h++)
	{
		CRC14 += msg7.Data[h];
	}
	for (int h = 0; h < 3; h++)
	{
		CRC14 += Position2.bytes[h];
	}
	msg8.StdId = 0x1001;
	msg8.ExtId = 0x00001008;
	msg8.IDE = CAN_Id_Extended;
	msg8.RTR = CAN_RTR_Data;
	msg8.DLC = 5;
	msg8.Data[0] = Position2.bytes[2];
	msg8.Data[1] = Position2.bytes[1];
	msg8.Data[2] = Position2.bytes[0];
	msg8.Data[3] = CRC14 & 0xff;
	msg8.Data[4] = 0x5C;
	//////////////////////////////////////
	MyCAN_Transmit(&msg5);
	MyCAN_Transmit(&msg6);
	Delay_ms(2);
	MyCAN_Transmit(&msg7);
	MyCAN_Transmit(&msg8);
};
// 简化重制版，更易懂
void CanMotor_CanSpeedMode_Run2(fp32 lspeed, fp32 rspeed, u16 accel)
{

	u8 Cdir;
	u8 Cdir2;
	u8 Accel = 10;
	u8 laddress = 0x02;
	u8 raddress = 0x01;
	if (lspeed < 0)
	{
		Cdir = 0;
		lspeed = -lspeed;
	}
	else
	{
		Cdir = 1;
	}
	if (rspeed < 0)
	{
		Cdir2 = 0;
		rspeed = -rspeed;
	}
	else
	{
		Cdir2 = 1;
	}
	CanMotor_CanSpeedMode_Run(laddress, accel, lspeed, Cdir, raddress, Accel, rspeed, Cdir2);
}
