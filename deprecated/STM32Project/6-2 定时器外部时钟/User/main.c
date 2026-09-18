#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"

uint16_t Num;
int main(void)
{
	// 初始化外设
	OLED_Init();  // 初始化 OLED 显示屏
	Timer_Init();  // 初始化 Timer 计时器
	
	OLED_ShowString(1, 1, "Num:");
	OLED_ShowString(2, 1, "Cnt:");
	while (1)
	{
		OLED_ShowNum(1, 5, Num, 5);
		OLED_ShowNum(2, 5, Timer_GetCounter(), 5);
	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Num ++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
