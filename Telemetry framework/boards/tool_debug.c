#include "tool_debug.h"

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
/* Keil ARMCC / AC6: printf 调用 fputc，extern "C" 防止 C++ 命名冲突 */
#ifdef __cplusplus
extern "C" {
#endif
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
	return ch;
}
#ifdef __cplusplus
}
#endif
#else
/* GCC (CubeIDE): printf 调用 __io_putchar */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
	return ch;
}
#endif
