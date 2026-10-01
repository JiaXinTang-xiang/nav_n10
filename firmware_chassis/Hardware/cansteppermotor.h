#ifndef __CANSTEPPERMOTOR_H
#define __CANSTEPPERMOTOR_H

#include "stdint.h"
#define fp32 float
#define u8 uint8_t
#define u32 uint32_t
#define u16 uint16_t

#pragma pack(push, 1) 

typedef union
	         {
		        u16 Acceleration;
		        u8 bytes2[2];
	         }converter_t1;
		 
typedef union
	{
		fp32 f_val; 
		u32 u32_val;
		u8 bytes[4];
	}converter_t2;	
	
typedef union
	         {
		        u16 u16_val;
		        u8 bytes3[2];
	         }converter_t3;

#pragma pack(pop) 
		 
void CanMotor_Init(void);
void CanMotor_CanSpeedMode_Run(u8 address,u16 Accel,fp32 onespeed,u8 adr1,         u8 address2,u16 Accel2,fp32 onespeed2,u8 adr2);
			 
/*				Make by 轩
 *
 * 函数名: CanMotor_CanSpeedMode_Run
 * 描述:   通过CAN总线控制两个步进电机以速度模式运行
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
void  CanMotor_CanPositionMode_Run(u8 address,u16 Accel,u16 onespeed1,u32 position1 ,u8 adr1,         u8 address2,u16 Accel2, u16 onespeed2,u32 position2,u8 adr2);

void CanMotor_CanSpeedMode_Run2(fp32 lspeed,fp32 rspeed,u16 accel);
#endif

 
