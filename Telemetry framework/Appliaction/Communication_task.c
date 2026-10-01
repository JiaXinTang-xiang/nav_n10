#include "communication_task.h"



Struct_Vision_Data Vision_Data = {0};
uint8_t Vision_Rx_State = 0;
uint8_t Vision_Rx_Index = 0;
uint8_t Vision_Rx_Count = 0;
uint8_t Vision_Rx_Digits[VISION_MAX_DIGITS];
uint8_t Vision_Rx_Checksum = 0;
void Vision_Input_Byte(uint8_t Data);

/**
 * @brief  Input one UART byte and synchronize a MaixCAM2 frame.
 * @note   Frame format: 12 | count | digit_1 ... digit_4 | xor | 5B.
 */
void Vision_Input_Byte(uint8_t Data)
{
    uint8_t i;

    switch (Vision_Rx_State)
    {
        case 0:
            if (Data == VISION_FRAME_HEAD)
            {
                Vision_Rx_State = 1;
            }
            break;

        case 1:
            if (Data <= VISION_MAX_DIGITS)
            {
                Vision_Rx_Count = Data;
                Vision_Rx_Index = 0;
                Vision_Rx_Checksum = Data;
                Vision_Rx_State = 2;
            }
            else
            {
                Vision_Data.Error_Count++;
                Vision_Rx_State = (Data == VISION_FRAME_HEAD) ? 1 : 0;
            }
            break;

        case 2:
            /* Valid digits are 1 through 8; unused digit positions are zero. */
            if (((Vision_Rx_Index < Vision_Rx_Count) &&
                 ((Data < 1U) || (Data > 8U))) ||
                ((Vision_Rx_Index >= Vision_Rx_Count) && (Data != 0U)))
            {
                Vision_Data.Error_Count++;
                Vision_Rx_State = (Data == VISION_FRAME_HEAD) ? 1 : 0;
                break;
            }

            Vision_Rx_Digits[Vision_Rx_Index++] = Data;
            Vision_Rx_Checksum ^= Data;
            if (Vision_Rx_Index >= VISION_MAX_DIGITS)
            {
                Vision_Rx_State = 3;
            }
            break;

        case 3:
            if (Data == Vision_Rx_Checksum)
            {
                Vision_Rx_State = 4;
            }
            else
            {
                Vision_Data.Error_Count++;
                Vision_Rx_State = (Data == VISION_FRAME_HEAD) ? 1 : 0;
            }
            break;

        case 4:
            if (Data == VISION_FRAME_TAIL)
            {
                Vision_Data.Digit_Count = Vision_Rx_Count;
                for (i = 0; i < VISION_MAX_DIGITS; i++)
                {
                    Vision_Data.Digits[i] = Vision_Rx_Digits[i];
                }
                Vision_Data.Result_Ready = 1;
                Vision_Data.Frame_Count++;
                Vision_Rx_State = 0;
            }
            else
            {
                Vision_Data.Error_Count++;
                Vision_Rx_State = (Data == VISION_FRAME_HEAD) ? 1 : 0;
            }
            break;

        default:
            Vision_Rx_State = 0;
            break;
    }
}

/**
 * @brief  UART receive DMA idle callback (USART1).
 * @param  Buffer  Receive data buffer
 * @param  Length  Receive data length
 */
void UART_Vision_Call_Back(uint8_t *Buffer, uint16_t Length)
{
	 uint16_t i;

    if ((Buffer == 0) || (Length == 0))
    {
        return;
    }

    Vision_Data.Byte_Count += Length;
    for (i = 0; i < Length; i++)
    {
        Vision_Input_Byte(Buffer[i]);
    }
}

uint8_t Vision_GetResultFlag(void)
{
    if (Vision_Data.Result_Ready == 0U)
    {
        return 0;
    }

    Vision_Data.Result_Ready = 0;
    return 1;
}

void UART_WireLess_Call_Back(uint8_t *Buffer, uint16_t Length)
{

}
	

