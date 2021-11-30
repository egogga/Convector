/*
 * 7led.c
 *
 *  Created on: 24 дек. 2020 г.
 *      Author: Professional
 */
#include "main.h"

void led(int8_t number, int8_t rank)
{
//	HAL_GPIO_WritePin(LED_Anode1_GPIO_Port,LED_Anode1_Pin,1);
//	HAL_GPIO_WritePin(LED_Anode2_GPIO_Port,LED_Anode2_Pin,1);
	if(rank == 1)
	{

				switch(number)
				{
				case -1:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,1);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,1);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,1);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,1);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,1);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				case 0:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,0);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,0);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,0);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,1);
					  break;
				case 1:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,1);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,1);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,1);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,1);
					  break;
				case 2:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,1);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,0);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,0);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,1);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				case 3:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,0);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,1);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				case 4:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,1);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,1);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,0);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				case 5:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,1);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,0);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,0);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				case 6:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,1);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,0);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,0);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,0);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				case 7:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,1);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,1);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,1);
					  break;

				case 8:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,0);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,0);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,0);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				case 9:
					  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,0);
					  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,0);
					  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,0);
					  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,0);
					  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
					  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,0);
					  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,0);
					  break;
				}
	}
	else if(rank == 2)
	{
		switch(number)
				{
				case 0:

					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,0);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,0);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,0);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,1);
					  break;
				case 1:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,1);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,1);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,1);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,1);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,1);
					  break;
				case 2:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,1);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,0);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,0);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,1);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,0);
					  break;
				case 3:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,0);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,1);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,1);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,0);
					  break;
				case 4:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,1);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,1);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,1);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,0);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,0);
					  break;
				case 5:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,1);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,0);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,1);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,0);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,0);
					  break;
				case 6:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,1);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,0);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,0);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,0);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,0);
					  break;
				case 7:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,1);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,1);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,1);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,1);
					  break;

				case 8:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,0);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,0);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,0);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,0);
					  break;
				case 9:
					  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,0);
					  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,0);
					  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,0);
					  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,0);
					  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,1);
					  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,0);
					  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,0);
					  break;
				}
	}
	else if(rank == 0)
	{
		//отключить все светодиоды
		  HAL_GPIO_WritePin(LED_a1_GPIO_Port,LED_a1_Pin,1);
		  HAL_GPIO_WritePin(LED_b1_GPIO_Port,LED_b1_Pin,1);
		  HAL_GPIO_WritePin(LED_c1_GPIO_Port,LED_c1_Pin,1);
		  HAL_GPIO_WritePin(LED_d1_GPIO_Port,LED_d1_Pin,1);
		  HAL_GPIO_WritePin(LED_e1_GPIO_Port,LED_e1_Pin,1);
		  HAL_GPIO_WritePin(LED_f1_GPIO_Port,LED_f1_Pin,1);
		  HAL_GPIO_WritePin(LED_g1_GPIO_Port,LED_g1_Pin,1);

		  HAL_GPIO_WritePin(LED_a2_GPIO_Port,LED_a2_Pin,1);
		  HAL_GPIO_WritePin(LED_b2_GPIO_Port,LED_b2_Pin,1);
		  HAL_GPIO_WritePin(LED_c2_GPIO_Port,LED_c2_Pin,1);
		  HAL_GPIO_WritePin(LED_d2_GPIO_Port,LED_d2_Pin,1);
		  HAL_GPIO_WritePin(LED_e2_GPIO_Port,LED_e2_Pin,1);
		  HAL_GPIO_WritePin(LED_f2_GPIO_Port,LED_f2_Pin,1);
		  HAL_GPIO_WritePin(LED_g2_GPIO_Port,LED_g2_Pin,1);

	}

}
