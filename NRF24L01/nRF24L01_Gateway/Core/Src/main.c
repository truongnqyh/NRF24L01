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
#include "NRF24L01.h"
#include "string.h"
#include "stdio.h"
#include "i2c-lcd.h"
#include "cJSON.h"
#include "cJSON_Utils.h"
#include <stdlib.h>
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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RxAdrdress[]={0x11,0xDD,0xCC,0xBB,0xAA};

uint8_t RxData[32];
uint8_t RxData1[32];
uint8_t TxData1[32];
float voltage_value,adc_value,moisture_value;
int moisturefinal=0;
int lcdset=1;
uint16_t rh_byte1,rh_byte2, temp_byte1,temp_byte2;
uint16_t sum,rh,temp,presence=0;
int temperature=0,humidity=0;

cJSON *str_json1, *str_json2, *str_json3,*str_ND1,*str_DA1,*str_DAD1,*str_ND2,*str_DA2,*str_DAD2,*str_ND3,*str_DA3,*str_DAD3;
uint8_t ND1_1,DA1_1,DAD1_1,ND1_2,DA1_2,DAD1_2,ND1_3,DA1_3,DAD1_3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while((__HAL_TIM_GET_COUNTER(&htim2))<time);
}

void set_pin_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void set_pin_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void dht11_init(void)
{
	set_pin_output(DHT11_GPIO_Port,DHT11_Pin);
	HAL_GPIO_WritePin(DHT11_GPIO_Port,DHT11_Pin,GPIO_PIN_RESET);
	HAL_Delay(25);
	HAL_GPIO_WritePin(DHT11_GPIO_Port,DHT11_Pin,GPIO_PIN_SET);
	delay(30);
	set_pin_input(DHT11_GPIO_Port,DHT11_Pin);
	
}
short check_response(){
  delay(40);
  if(!HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin)){                     // Read and test if connection pin is low
    delay(80);
    if(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin)){                    // Read and test if connection pin is high
      delay(50);
      return 1;}
    }
	return 0;
}

uint8_t read(void)
{
	uint8_t i,j;
	for(j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port,DHT11_Pin)));
		delay(40);
		if(!HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))
		{
			i&= ~(1<<(7-j));
		}
		else
		{
			i|= (1<<(7-j));
		}
		while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port,DHT11_Pin)));
	
  }
			return i	;	
}		
void ParseData(char * TxData, uint8_t temp, uint8_t humi, uint8_t moil){
	char Temperature[10];
	char Humidity[10];
	char Doamdat[10];
	for (int i=0;i<strlen(TxData);i++){
		TxData[i]=0;
	}
	sprintf(Doamdat,"%d",moil);
	sprintf(Temperature,"%d",temp);
	sprintf(Humidity,"%d",humi);
	
	strcat(TxData,"{\"T3\":\"");
	strcat(TxData,Temperature);
	strcat(TxData,"\",");
	
	strcat(TxData,"\"H3\":\"");
	strcat(TxData,Humidity);
	strcat(TxData,"\",");
	
	strcat(TxData,"\"M3\":\"");
	strcat(TxData,Doamdat);
	strcat(TxData,"\"}");
}
void getdata(){
	HAL_TIM_Base_Start(&htim2);
	dht11_init();
	presence=check_response();
	rh_byte1=read();
	rh_byte2=read(); 
	temp_byte1=read(); 
	temp_byte2=read(); 
	sum=read(); 
	temp=temp_byte1; 
	rh=rh_byte1; 
	temperature=(int) temp;//temp
	humidity=(int) rh;//humi
	adc_value= HAL_ADC_GetValue(&hadc1);
	voltage_value=(adc_value/4096)*3.3;
	moisture_value= (100-(voltage_value/3.3)*100);
	moisturefinal = (int)moisture_value;//mois
}
void Lcdtrangchu(){
	lcd_clear();
	lcd_put_cur(0,0);
	lcd_send_string("Trang chu");
	lcd_put_cur(1,0);
	lcd_send_string("Smart Garden");
	lcd_put_cur(2,0);
	lcd_send_string("Moi chon nut");
	lcd_put_cur(3,0);
	lcd_send_string("Trai           Phai");
	
}
void lcdnode1(){
	char send[20];
	char send1[20];
	char send2[20];
	lcd_clear();
	lcd_put_cur(0,0);
	lcd_send_string("NODE1");
	lcd_put_cur(1,0);
	sprintf(send,"Temp: %d ", ND1_1);
	lcd_send_string(send);
	lcd_put_cur(2,0);
	sprintf(send1,"Humi: %d ", DA1_1);
	lcd_send_string(send1);
	lcd_put_cur(3,0);
	sprintf(send2,"Mois: %d ", DAD1_1);
	lcd_send_string(send2);
}
void lcdnode2(){
	char send[20];
	char send1[20];
	char send2[20];
	lcd_clear();
	lcd_put_cur(0,0);
	lcd_send_string("NODE2");
	lcd_put_cur(1,0);
	sprintf(send,"Temp: %d ", ND1_2);
	lcd_send_string(send);
	lcd_put_cur(2,0);
	sprintf(send1,"Humi: %d ", DA1_2);
	lcd_send_string(send1);
	lcd_put_cur(3,0);
	sprintf(send2,"Mois: %d ", DAD1_2);
	lcd_send_string(send2);
}
void lcdnode3(){
	char send[20];
	char send1[20];
	char send2[20];
	lcd_clear();
	lcd_put_cur(0,0);
	lcd_send_string("NODE3");
	lcd_put_cur(1,0);
	sprintf(send,"Temp: %d ", ND1_3);
	lcd_send_string(send);
	lcd_put_cur(2,0);
	sprintf(send1,"Humi: %d ", DA1_3);
	lcd_send_string(send1);
	lcd_put_cur(3,0);
	sprintf(send2,"Mois: %d ", DAD1_3);
	lcd_send_string(send2);
}
void XuLyJSON1(char *data1)
{
	
	str_json1 = cJSON_Parse(data1);
	if (!str_json1)
  {
			printf("JSON1 error\r\n"); 
			return;
  }
	
	else
	{
		printf("JSON OKE\r\n"); 
		//{"Bom":"0"} {"Bom":"1"}
		str_ND1 = cJSON_GetObjectItem(str_json1, "T1");
		str_DA1 = cJSON_GetObjectItem(str_json1, "H1");
		str_DAD1 = cJSON_GetObjectItem(str_json1, "M1");
      // node1
		if(str_ND1->type == cJSON_String){
			ND1_1=atoi(str_ND1->valuestring);
		}
		if(str_DA1->type == cJSON_String){
			DA1_1=atoi(str_DA1->valuestring);
		}
		if(str_DAD1->type == cJSON_String){
			DAD1_1=atoi(str_DAD1->valuestring);
		}
		cJSON_Delete(str_json1);

	}
}
void XuLyJSON2(char *data2)
{
	
	str_json2 = cJSON_Parse(data2);
	if (!str_json2)
  {
			printf("JSON2 error\r\n"); 
			return;
  }
	
	else
	{
		printf("JSON OKE\r\n"); 
		//{"Bom":"0"} {"Bom":"1"}
		str_ND2 = cJSON_GetObjectItem(str_json2, "T2");
		str_DA2 = cJSON_GetObjectItem(str_json2, "H2");
		str_DAD2 = cJSON_GetObjectItem(str_json2, "M2");
      // node1
		if(str_ND2->type == cJSON_String){
			ND1_2=atoi(str_ND2->valuestring);
		}
		if(str_DA2->type == cJSON_String){
			DA1_2=atoi(str_DA2->valuestring);
		}
		if(str_DAD2->type == cJSON_String){
			DAD1_2=atoi(str_DAD2->valuestring);
		}
		cJSON_Delete(str_json2);

	}
}
void XuLyJSON3(char *data3)
{
	
	str_json3 = cJSON_Parse(data3);
	if (!str_json3)
  {
			printf("JSON3 error\r\n"); 
			return;
  }
	
	else
	{
		printf("JSON OKE\r\n"); 
		//{"Bom":"0"} {"Bom":"1"}
		str_ND3 = cJSON_GetObjectItem(str_json3, "T3");
		str_DA3 = cJSON_GetObjectItem(str_json3, "H3");
		str_DAD3 = cJSON_GetObjectItem(str_json3, "M3");
      // node1
		if(str_ND3->type == cJSON_String){
			ND1_3=atoi(str_ND3->valuestring);
		}
		if(str_DA3->type == cJSON_String){
			DA1_3=atoi(str_DA3->valuestring);
		}
		if(str_DAD3->type == cJSON_String){
			DAD1_3=atoi(str_DAD3->valuestring);
		}
		cJSON_Delete(str_json3);

	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
     #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
     #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_ResumeTick();
	
	if(GPIO_Pin == GPIO_PIN_0){
		int a;
		
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
		if(IsDataAvailable(1)==1)
			{
				
				NRF24_Receive(RxData);
			}
		if(IsDataAvailable(2)==1)
			{
				__HAL_TIM_SetCounter(&htim4,0);
				NRF24_Receive(RxData1);
				getdata();
				ParseData((char *)TxData1,temperature, humidity,moisturefinal);
				printf("%s\n",RxData);
				XuLyJSON1((char *)RxData);
				XuLyJSON2((char *)RxData1);
				XuLyJSON3((char *)TxData1);
				a = __HAL_TIM_GetCounter(&htim4);
				printf("%d\n",a);
			}
	}
	else if(GPIO_Pin == GPIO_PIN_3)
	{
		int b;
		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==0);
		__HAL_TIM_SetCounter(&htim4,0);
		lcdset+=1;
		if(lcdset == 5)
		{
			lcdset=1;
		}
		switch (lcdset)
		{
		case 1:
			Lcdtrangchu();
		  break;
    case 2:
			lcdnode1();
      break;
    case 3:
			lcdnode2();
      break;
		case 4:
			lcdnode3();
    default:
			break;
    }
		b = __HAL_TIM_GetCounter(&htim4);
		printf("%d\n",b); 
	}
	else if(GPIO_Pin == GPIO_PIN_1)
	{
		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0);
		lcdset-=1;
		if(lcdset == 0)
		{
			lcdset=4;
		}
		switch (lcdset)
		{
		case 1:
			Lcdtrangchu();
		  break;
    case 2:
			lcdnode1();
      break;
    case 3:
			lcdnode2();
      break;
		case 4:
			lcdnode3();
    default:
			break;
    }
	}
//	else if(GPIO_Pin == GPIO_PIN_4){
//		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==0);
//		
//	}
}


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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim4);
	HAL_ADC_Start(&hadc1);
	lcd_init();
	Lcdtrangchu();
	NRF24_Init();
	NRF24_RxMode(RxAdrdress, 10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
