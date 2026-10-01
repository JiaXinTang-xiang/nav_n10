#ifndef __MYCAN__H
#define __MYCAN__H

#include "bsp.h"

void MyCAN_Init(void);
void MyCAN_Transmit_Double(CanTxMsg *TxMessage , CanTxMsg *TxMessage2);
void MyCAN_Transmit(CanTxMsg *TxMessage );
uint8_t MyCAN_ReceiveFlag(void);
void MyCAN_Receive(CanRxMsg *RxMessage);




#endif
 
  
