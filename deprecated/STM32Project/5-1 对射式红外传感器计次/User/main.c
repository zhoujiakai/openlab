#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "CountSensor.h"

int main(void)
{
	// 初始化外设
	OLED_Init();  // 初始化 OLED 显示屏
	CountSensor_Init();  // 初始化对射式红外传感器
	
	OLED_ShowString(1, 1, "Count:");
	while (1)
	{
		OLED_ShowNum(1, 7, CountSensor_Get(), 5);
	}
}
