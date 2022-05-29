#include "stm32f3xx_hal.h"
#include "var_extern.h"
#include "defs.h"

extern TIM_HandleTypeDef htim2;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  Small_ticks++;
	Small_tick_over=1;
	if(Small_ticks >= 5)
	{
		Small_ticks=0;
		One_tick_over=1;
		Ticks++;
		if(Ticks % 10 ==0)
			Ms50_over=1;
		
		if((Ticks % 20)==0)
			Ms100_over = 1;
		
		if(Ticks == 100)
			Half_sec_over=1;
		
		if(Ticks >= 200)
		{
			Ticks=0;
			Secs++;
			Half_sec_over=1;
			One_sec_over=1;
			
			if(Secs >= 60)
			{
				Secs=0;
				One_min_over=1;
			}
		}
	}
}

