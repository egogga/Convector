/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

float ADC_Data; //переменная в которую записываются данные с АЦП
float temperature,tempData,prevTemperature,k,TimerAccelerator;
float *addr_ADC_Data;
uint8_t SensButton1,SensButton2,SensButton3;
uint8_t i,j, IJ,m,n,hysteresis;
int myDelay,TimeCounter,LimitTimeCounter,temperatureCounter,temperatureCounterLimit,intTemperature;
_Bool mode,Button1Pressed,Button2Pressed,Button3Pressed;

//int *addr_VrefInt;
//int vref;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void Beep(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	k = 0.3;// коэффициент для фильтра сколegьзящего среднего, чем больше k тем быстрее показания догоняют реально значение
	prevTemperature=22;// [град],предыдущее значение температуры для фильтрации скользящим средним
	myDelay=10;// отвечает за частоту опроса АЦП и кнопок
	TimeCounter=0;// счетчик, чтобы при регулеровке температуры кнопками показывать пользовательскую температуру, а потом через
	//несколько секунд уже реальную
	LimitTimeCounter=(int)(3000/myDelay);// при нажатии кнопки + или - подождать 3000мс прежде чем показать реальную температуру
	mode=0;// режим вкл или выкл
	ADC_Data=0; //переменная в которую пишутся данные с АЦП
//	HAL_GPIO_WritePin(LED_Anode1_GPIO_Port,LED_Anode1_Pin,0);
//	HAL_GPIO_WritePin(LED_Anode2_GPIO_Port,LED_Anode2_Pin,0);
	i=2;//старший разряд числа IJ
	j=2;// младший разряд числа IJ
	temperature=22;//начальная установка РЕАЛЬНОЙ температуры,чтобы при включении до реальных измерений
	//не щелкало рэле
	m=2;//старший разряд числа
	n=2;//старший разряд числа
	IJ=22; //[град], установленная пользователем температура.
	hysteresis=1; //,[град]гистерезис пользовательской температуры для исключения дребезга рэле
	temperatureCounter=0;
	temperatureCounterLimit=(int)(300/myDelay);//раз в 300мс опрашивать датчик температуры, а так же
	//кнопки + и - (для отработки долгого нажатия)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(mode) // если конвектор включен
	  {
		  if(TimeCounter<LimitTimeCounter)// если не прошло 3 секунды после установки температуры пользователем
		  {	// то показать устанновленную пользователем температуру
			  i=(int)(IJ/10);//вычисление старшего разряда для отображения на дисплее
			  j=(int)(IJ-i*10);//вычисление младшего разряда для отображения на дисплее
		  	  led(i,1);	// показать старший разряд установленной пользователем температуры
		  	  led(j,2);// показать младший разряд установленной пользователем температуры
		  	  TimeCounter++; // запустить счетчик нажатой недавно кнопки
		  }
		  else if((!Button2Pressed)&&(!Button3Pressed))// если 3 секунды прошли после последнего нажатия кнопок
		  {// то показать реальную температуру
			  led(m,1);			// показать старший разряд реальной температуры
			  led(n,2);  // показать младший разряд реальной температуры
			  Button2Pressed=0; // вернуть признак нажатой недавно кнопки в исходное состояние
			  Button3Pressed=0; // вернуть признак нажатой недавно кнопки в исходное состояние

		  }


		  //Регулировка температуры
		  if(temperature<IJ-hysteresis) HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin,1);//если температура ниже установленной IJ, то включить рэле
		  else if(temperature>IJ+hysteresis) HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin,0);// если выше, то отключить рэле

		  if(temperatureCounter<temperatureCounterLimit) temperatureCounter++;//лимит счетчика зависит от задержки цикла
		  else//чтобы получить считывание температуры раз в 300мс
		  {
			  //Замер реальной температуры
			  HAL_ADC_Start_IT(&hadc1); // запустить АЦП для измерения температуры
			  addr_ADC_Data=&ADC_Data; ////взять адрес переменной ADC_Data
			  tempData=*addr_ADC_Data;//взять данные по адресу переменной ADC_Data
			  //Расчет температуры с датчика
			  temperature=(((tempData*1.16)/64)-0.52)*100; //Vout= (Tc*Ta-0.5) - формула из ДШ на датчик температуры.
			  //1.16 -почему-то опорное напряжение хотя должно быть 3, 2.5 или 2.048 вольт, или хотя бы 1.2
			  //64 - почему то как будто разрядность АЦП хотя на самом деле 256
			  //0.52 это 0.5 из формулы - поправочный коэффициент - значение Vout при 0 градусов
			  //100 - это 1/0,01[В/С] или 1/10[мВ/С] , где 10[мВ/С] это крутизна характеристики датчика температуры
			  prevTemperature +=(temperature - prevTemperature)*k; //фильтр скользящего среднего
			  //Тут происходит округление реальной температуры для отображении на дисплее
			  intTemperature=(int)prevTemperature;// делаем из float температуры целое число
			  if(intTemperature>=0)
			  {
				  m=(int)(intTemperature/10); //старший разряд числа температуры это целочое число поделенное на 10 и
				  // с отброшенной дробной частью при помощи (int). Например 27/10=2.7 Тогда (int)2.7=2
				  n=(int)(intTemperature-m*10);//С младшим разрядом примерно то же самое. �?з полной температуры отнимаем десятку, умноженную на m.
				  //Где m это зн старшего разряда. Например 27-2*10=7. �? на всякий случай превращаем снова в int.
			  }
			  else if(intTemperature>=-9)
			  {
				  m=-1;
				  n=abs(intTemperature);
			  }


			  temperatureCounter=0;

			  //Кнопки нужно сбросить чтобы  кнопки переключались и удержанием(плюс к тому что они работают по однократному нажатию
			  Button2Pressed=0; // вернуть признак нажатой недавно кнопки в исходное состояние
			  Button3Pressed=0; // вернуть признак нажатой недавно кнопки в исходное состояние
		  }

  		  SensButton3 = HAL_GPIO_ReadPin(SensBut3_GPIO_Port, SensBut3_Pin);
  		  if((SensButton3 == 1)&&(!Button3Pressed))
  			 {// по нажатию кнопки 3(+ температура)
  			  	  if(IJ<40)//если установка температуры не выше 40 градусов то
  			  	  {
  			  		  IJ++; //прибавить установленную температуру
  			  	  }
  			  	  Beep(); //функция Бип - это пищалка
  				  Button3Pressed=1;// признак нажатой кнопки нужен для установки задержки(запуска счетчика TimeCounter)
  				  TimeCounter=0;// вернуть таймер нажатой недавно кнопки в исходное состояние
  			 }
  		  else if((SensButton3 == 0)&&(Button3Pressed))// если кнопка была отпущена , и стоит признак ее недавнего нажатия
  		  {
  			 Button3Pressed=0;//, то обнулить признак недавнего нажатия кнопки
  		  }//То есть кнопкой можно восопользоваться еще раз только если ее отпустить и нажать снова.
		  //При длительном удержании она не сработает

  		  SensButton2 = HAL_GPIO_ReadPin(SensBut2_GPIO_Port, SensBut2_Pin);
  		  if((SensButton2 == 1)&&(!Button2Pressed))
  			 {	// по нажатию кнопки 2(- температура)
  			  	  if(IJ != 0)
  			  	  {
					  Beep();//функция Бип - это пищалка
					  IJ--;//убавить установленную температуру
					  Button2Pressed=1;// признак нажатой кнопки нужен для установки задержки(запуска счетчика TimeCounter)
					  TimeCounter=0;// вернуть таймер нажатой недавно кнопки в исходное состояние
  			  	  }
  			 }
  		  else if((SensButton2 == 0)&&(Button2Pressed))// если кнопка была отпущена , и стоит признак ее недавнего нажатия
				{
  			  	  	  Button2Pressed=0;//, то обнулить признак недавнего нажатия кнопки
				}//То есть кнопкой можно восопользоваться еще раз только если ее отпустить и нажать снова.
			  //При длительном удержании она не сработает



		  // Во включенном режиме кнопка 1 (ВКЛ/ВЫКЛ)
		  SensButton1 = HAL_GPIO_ReadPin(SensBut1_GPIO_Port, SensBut1_Pin);
		  if((SensButton1 == 1)&&(!Button1Pressed))//Button1Pressed -  от ложного срабатывания кнопки
			  {
			  	  	Beep();//Должна сделать Бип
			  	  	mode=0;// установить режим работы modeOFF
//					HAL_GPIO_TogglePin(LED_Anode1_GPIO_Port,LED_Anode1_Pin); //отключить светодиоды
//					HAL_GPIO_TogglePin(LED_Anode2_GPIO_Port,LED_Anode2_Pin);
					HAL_GPIO_WritePin(Relay_GPIO_Port,Relay_Pin,0);  //отключить рэле
				//	HAL_Delay(500);// задержка для того чтобы успеть убрать палец от кнопки
					Button1Pressed=1; // кнопку недавно нажмали
					led(0,0);	// отключить все светодиоды
			  }
			  else if((SensButton1 == 0)&&(Button1Pressed))// если кнопка была отпущена , и стоит признак ее недавнего нажатия
			  {
				  Button1Pressed=0; //, то обнулить признак недавнего нажатия кнопки
			  }//То есть кнопкой можно восопользоваться еще раз только если ее отпустить и нажать снова.
		  //При длительном удержании она не сработает


	  }
	  else // if mode = 0 -то есть если конвектор выключен то
	  {
		  SensButton1 = HAL_GPIO_ReadPin(SensBut1_GPIO_Port, SensBut1_Pin);
		  if((SensButton1 == 1)&&(!Button1Pressed))
			  {
			  	  	Beep();//сделать Бип
			  		mode=1; //включить режиме modeOn то есть включить конвектор.
//					HAL_GPIO_TogglePin(LED_Anode1_GPIO_Port,LED_Anode1_Pin);//переключить светодиоды
//					HAL_GPIO_TogglePin(LED_Anode2_GPIO_Port,LED_Anode2_Pin);
					HAL_GPIO_WritePin(Relay_GPIO_Port,Relay_Pin,0);//отключить рэле
					//HAL_Delay(500);// задержка для того чтобы успеть убрать палец от кнопки
					Button1Pressed=1;// кнопку недавно нажмали

			  }
		  else if((SensButton1 == 0)&&(Button1Pressed))// если кнопка была отпущена , и стоит признак ее недавнего нажатия
		  {
			  Button1Pressed=0;//, то обнулить признак недавнего нажатия кнопки
		  }//То есть кнопкой можно восопользоваться еще раз только если ее отпустить и нажать снова.
		  //При длительном удержании она не сработает
	  }
	  HAL_Delay(myDelay);


  }
    /* USER CODE END WHILE */
}
    /* USER CODE BEGIN 3 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	ADC_Data=HAL_ADC_GetValue(hadc1); //получение переменной из буфера ацп
	HAL_ADC_Start_IT(hadc1); // перезапуск ацп
}

void Beep(void)
{
	uint16_t BeepCounter;
	BeepCounter=0;
	for(BeepCounter=0;BeepCounter<100;BeepCounter++)// регулировка длительности импульса Бипа
	{
	  HAL_TIM_Base_Start(&htim6);
	  HAL_TIM_Base_Start_IT(&htim6);
	  //HAL_TIM_Base_Start(&htim7);
	 // HAL_TIM_Base_Start_IT(&htim7);
	}
	HAL_TIM_Base_Stop(&htim6);
	HAL_TIM_Base_Stop_IT(&htim6);

}

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 124;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 3999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_f1_Pin|LED_e1_Pin|LED_c1_Pin|LED_a1_Pin 
                          |LED_b1_Pin|LED_e2_Pin|LED_d2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_f2_Pin|LED_g2_Pin|LED_a2_Pin|LED_c2_Pin 
                          |LED_b2_Pin|LED_d1_Pin|LED_g1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(piezo_p_GPIO_Port, piezo_p_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_f1_Pin LED_e1_Pin LED_c1_Pin LED_a1_Pin 
                           LED_b1_Pin LED_e2_Pin LED_d2_Pin Relay_Pin */
  GPIO_InitStruct.Pin = LED_f1_Pin|LED_e1_Pin|LED_c1_Pin|LED_a1_Pin 
                          |LED_b1_Pin|LED_e2_Pin|LED_d2_Pin|Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_f2_Pin LED_g2_Pin LED_a2_Pin LED_c2_Pin 
                           LED_b2_Pin piezo_p_Pin LED_d1_Pin LED_g1_Pin */
  GPIO_InitStruct.Pin = LED_f2_Pin|LED_g2_Pin|LED_a2_Pin|LED_c2_Pin 
                          |LED_b2_Pin|piezo_p_Pin|LED_d1_Pin|LED_g1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SensBut1_Pin SensBut2_Pin SensBut3_Pin */
  GPIO_InitStruct.Pin = SensBut1_Pin|SensBut2_Pin|SensBut3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
