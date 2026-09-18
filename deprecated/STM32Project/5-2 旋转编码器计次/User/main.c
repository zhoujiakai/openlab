#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Encoder.h"

int16_t Num;
int main(void)
{
	// 初始化外设
	OLED_Init();  // 初始化 OLED 显示屏
	Encoder_Init();  // 初始化 旋转编码器

	
	OLED_ShowString(1, 1, "Num:");
	while (1)
	{
		Num += Encoder_Get();
		OLED_ShowSignedNum(1, 5, Num, 5);
	}
}
