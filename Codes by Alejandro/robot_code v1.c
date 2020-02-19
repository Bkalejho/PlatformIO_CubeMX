
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define gpio_sens GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_15
#define max 90
#define giro 100
#define casos_t sensores==0xFF||sensores==0x7F||sensores==0x3F||sensores==0x1F||sensores==0x0F||sensores==0x07||sensores==0xFC||sensores==0xF8||sensores==0xF0||sensores==0x00
#define casos_d sensores==0xFF||sensores==0x7F||sensores==0x3F||sensores==0x1F||sensores==0x0F||sensores==0x07
#define casos_i sensores==0xFC||sensores==0xF8||sensores==0xF0

uint8_t sensores=0, RX[40]={0,0}, TX[40]={0,0}, flag=0, arreglo_1[100], arreglo_2[100], m=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void sensors_in(uint32_t sensors);
void sensors_out(uint32_t sensors);
uint8_t leer_sensores(void);
void seguidor(uint8_t dato);
void dere_atras(void);
void dere_adelante(void);
void izq_atras(void);
void izq_adelante(void);
void parar(void);
void motor_dere(uint8_t DC);
void motor_izq(uint8_t DC);
void adelante(void);
void giro_dere(void);
void giro_izq(void);
void exploracion(void);
void mini_adelante(void);
char reglas(int i);
void aprendizaje(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
	memset((char *)arreglo_1,'\0',strlen((char *)arreglo_1));
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_UART_Receive(&huart6, RX, 1, 3000);
		if(RX[0]!=0){
			HAL_UART_Transmit(&huart6, RX, 1, 3000);
			if(RX[0]=='Y'){
				flag=1;
				adelante();
			}
			if(RX[0]=='Z'){
				flag=2;
				adelante();
				m=0;
				HAL_UART_Transmit(&huart6, arreglo_1,strlen((char *)arreglo_1),200000);
			}
			if(RX[0]=='X'){
				flag=0;
				m=0;
				memset((char *)arreglo_1,'\0',strlen((char *)arreglo_1));
			}
			RX[0]=0;
		}
		while(flag==1){
			sensores=leer_sensores();
			if(casos_t){
				exploracion();
			}else{
				seguidor(sensores);
			}
			HAL_UART_Receive(&huart6, RX, 1, 3000);
			if(RX[0]!=0){
				HAL_UART_Transmit(&huart6, RX, 1, 3000);
				if(RX[0]=='X'){
					flag=0;
					m=0;
					memset((char *)arreglo_1,'\0',strlen((char *)arreglo_1));
					parar();
				}
				RX[0]=0;
			}
		}
		
		while(flag==2){
			sensores=leer_sensores();
			if(casos_t){
				parar();
				HAL_Delay(200000);
				mini_adelante();
				HAL_Delay(75000);
				parar();
				sensores=leer_sensores();
				if(sensores==0xFF){
					mini_adelante();
					HAL_Delay(75000);
					parar();
					flag=0;
					sprintf((char *)TX," Fin ");
					HAL_UART_Transmit(&huart6, TX,strlen((char *)TX),200000);
				}
				TX[0]=arreglo_1[m];
				TX[1]=' ';
				HAL_UART_Transmit(&huart6, TX,2,200000);
				switch(arreglo_1[m]){
					case 'R':
						giro_dere();
						parar();
						HAL_Delay(200000);
						m++;
						break;
					case 'S':
						seguidor(sensores);
						m++;
					break;
					case 'L':
						giro_izq();
						parar();
						HAL_Delay(200000);
						m++;
					break;
					case 'B':
						giro_izq();
						parar();
						HAL_Delay(200000);
						m++;
					break;
				}
			}else{
				seguidor(sensores);
			}
			HAL_UART_Receive(&huart6, RX, 1, 3000);
			if(RX[0]!=0){
				HAL_UART_Transmit(&huart6, RX, 1, 3000);
				if(RX[0]=='X'){
					flag=0;
					m=0;
					parar();
				}
				RX[0]=0;
			}
		}
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE9 PE11 PE12 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//.............................Poner pines de los sensores de entrada...........................
void sensors_in(uint32_t sensors){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = sensors;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
}
//.............................Poner pines de los sensores de salida............................
void sensors_out(uint32_t sensors){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = sensors;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}
//........................................Leer sensores..........................................
uint8_t leer_sensores(void){
	uint8_t temp=0;
	
	sensors_out(gpio_sens);
	HAL_GPIO_WritePin(GPIOD, gpio_sens,GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOD, gpio_sens,GPIO_PIN_RESET);
	sensors_in(gpio_sens);
	HAL_Delay(500);
	temp = (GPIOD->IDR&0x80)|((GPIOD->IDR&0x08)<<3)|((GPIOD->IDR&0x02)<<4)|
	((GPIOD->IDR&0x01)<<4)|((GPIOD->IDR&0x04)<<1)|((GPIOD->IDR&0x40)>>4)|((GPIOD->IDR&0x800)>>10)|
	((GPIOD->IDR&0x8000)>>15);
	
	return temp;
}
//......................................Seguidor de linea........................................
void seguidor(uint8_t dato){
	adelante();
	switch(dato){
		case 0x80:
			motor_dere(max*0.6);
			motor_izq(max*0);
			do{
				sensores=leer_sensores();
			}while(sensores!=0xC0);
			break;
		case 0xC0:
			motor_dere(max*0.9);
			motor_izq(max*0.07);
			break;
		case 0x60:
			motor_dere(max*0.8);
			motor_izq(max*0.37);
			break;
		case 0x30:
			motor_dere(max*0.89);
			motor_izq(max*0.67);
			break;
		case 0x18:
			motor_dere(max*0.9);
			motor_izq(max*0.87);
			break;
		case 0x0C:
			motor_dere(max*0.7);
			motor_izq(max*0.86);
			break;
		case 0x06:
			motor_dere(max*0.4);
			motor_izq(max*0.77);
			break;
		case 0x03:
			motor_dere(max*0.1);
			motor_izq(max*0.87);
			break;
		case 0x01:
			motor_dere(max*0);
			motor_izq(max*0.6);
			do{
				sensores=leer_sensores();
			}while(sensores!=0x03);
			break;
		
		default:
			break;
	}
}
//.....................................sentido de giro..........................................
void dere_atras(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
}
void dere_adelante(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
}
void izq_atras(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}
void izq_adelante(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}
//........................................parar.............................................
void parar(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
}
//........................................adelante.............................................
void adelante(void){
	dere_adelante();
	izq_adelante();
//	motor_dere(max*0.4);
//	motor_izq(max*0.4);
}
//........................................giros.............................................
void giro_dere(void){
	izq_adelante();
	dere_atras();
	motor_dere(giro*0.4);
	motor_izq(giro*0.4);
	HAL_Delay(50000);
	do{
		sensores=leer_sensores();
	}while(sensores!=0x01);
}

void giro_izq(void){
	izq_atras();
	dere_adelante();
	motor_dere(40);
	motor_izq(40);
	HAL_Delay(50000);
	do{
		sensores=leer_sensores();
	}while(sensores!=0x80);
}
//........................................Duty motores.............................................
void motor_dere(uint8_t DC){
	TIM_OC_InitTypeDef sConfigOC;
	uint16_t temp=0;
	
	temp=DC;
	temp*=5;
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = temp;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}
void motor_izq(uint8_t DC){

	TIM_OC_InitTypeDef sConfigOC;
	uint16_t temp=0;
	
	temp=DC;
	temp*=5;
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = temp;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
}
//........................................exploración.............................................	
void exploracion(void){
		HAL_Delay(20000);
		sensores=leer_sensores();
		if(casos_d){
			parar();
			HAL_Delay(200000);
			mini_adelante();
			HAL_Delay(75000);
			parar();
			sensores=leer_sensores();
			if(sensores==0xFF){
				mini_adelante();
				HAL_Delay(75000);
				parar();
				flag=0;
				HAL_UART_Transmit(&huart6, arreglo_1,strlen((char *)arreglo_1),200000);
//				aprendizaje();
				TX[0]=' ';
				TX[1]='F';
				TX[2]=' ';
				HAL_UART_Transmit(&huart6, TX,3,10000);
				aprendizaje();
				HAL_UART_Transmit(&huart6, arreglo_1,strlen((char *)arreglo_1),200000);
				return;
			}
			giro_dere();
			parar();
			HAL_Delay(200000);
			arreglo_1[m]='R';
			m++;
		}else if(casos_i){
			parar();
			HAL_Delay(200000);
			mini_adelante();
			HAL_Delay(75000);
			parar();
			sensores=leer_sensores();
			if(sensores!=0x00){
				seguidor(sensores);
				arreglo_1[m]='S';
				m++;
			}else{
				giro_izq();
				parar();
				HAL_Delay(200000);
				arreglo_1[m]='L';
				m++;
			}
		}else if(sensores==0x00){
			parar();
			HAL_Delay(200000);
			mini_adelante();
			HAL_Delay(75000);
			parar();
			giro_izq();
			parar();
			HAL_Delay(200000);
			arreglo_1[m]='B';
			m++;
		}else{
			seguidor(sensores);
		}
}
// reglas de aprenizaje ........................................................
char reglas(int i){
	if(arreglo_1[i-1]=='R'){
		if(arreglo_1[i+1]=='R'){
			return 'S';
		}
		if(arreglo_1[i+1]=='S'){
			return 'L';
		}
		if(arreglo_1[i+1]=='L'){
			return 'B';
		}
	}
	if(arreglo_1[i-1]=='S'){
		if(arreglo_1[i+1]=='S'){
			return 'B';
		}
		if(arreglo_1[i+1]=='R'){
			return 'L';
		}
		if(arreglo_1[i+1]=='L'){
			return 'B';
		}
	}
	if(arreglo_1[i-1]=='L'){
		return 'B';
	}
	
	return 0;
}	
//........................................mini_adelante.............................................	
void mini_adelante(void){
	dere_adelante();
	izq_adelante();
	motor_dere(giro*0.4);
	motor_izq(giro*0.4);
}
// aprendizaje .................................................................
void aprendizaje(void){
    
  char x=0;
	int i=0, j=0, flag=1, k=0;
	j=strlen((char *) arreglo_1);
	strcpy((char *) arreglo_2,(char *) arreglo_1);	
	while(flag){
		strcpy((char *) arreglo_1,(char *) arreglo_2);
		memset((char *) arreglo_2,'\n',strlen((char *) arreglo_2));
		flag=0;
		j=strlen((char *) arreglo_1);
		for(i=1;i<j-1;i++){
			if(arreglo_1[i]=='B'){
				flag++;
				x=reglas(i);
				if(i==1){
					arreglo_2[0]=x;
					strcpy((char *) arreglo_2+1,(char *) arreglo_1+i+2);
					break;
				}
				else{
					for(k=0;k<=i-2;k++){
						arreglo_2[k]=arreglo_1[k];
					}
					arreglo_2[k]=x;
					strcpy((char *) arreglo_2+k+1,(char *) arreglo_1+i+2);
					break;
				}
			}
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
