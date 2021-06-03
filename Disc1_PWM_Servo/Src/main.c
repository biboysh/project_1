/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "angle.h"
#include "DeQueue.h"
#include "line.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  TRUE  =       1,
  FALSE =       0
}BOOL;

typedef enum{
  ARM_UP        =       00U,
  ARM_DOWN      =       !ARM_UP
}ARM_DETC;

typedef enum{
  START,
  CONTINUE,
  STOP
}sig_type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROGRAM 2

#define _PROJEDCT 1
//#define FALSE 0
//#define TRUE !FALSE
#define MIN_ANG 500
#define MAX_ANG MIN_ANG + 2000
#define RESOLUTION (MAX_ANG - MIN_ANG)/180.0
#define ANG_SIZE 7

#define INIT_ANG 45.0

#define BUFF_SIZE 20


#define START_X -146.0
#define START_Y 32.0

#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET

#define ENA ENA_GPIO_Port,ENA_Pin
#define ENB ENB_GPIO_Port,ENB_Pin

#define IN1 IN1_GPIO_Port,IN1_Pin
#define IN2 IN2_GPIO_Port,IN2_Pin
#define IN3 IN3_GPIO_Port,IN3_Pin
#define IN4 IN4_GPIO_Port,IN4_Pin

#define LED1 LED1_GPIO_Port,LED1_Pin
#define LED2 LED2_GPIO_Port,LED2_Pin

#define WritePin(x,y) HAL_GPIO_WritePin(x,y)

//typedef unsigned int BOOL;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BOOL dir = TRUE;
BOOL Ready_To_Rx = FALSE;
DEG position = {0.0,0.0};
XY before = {0.0,0.0};
int leng = 0;
Dequeue queue;

char move_str[30] = "0 0 1";
int len = 0;
int k = 0;
static XY goal = {0,0};

ARM_DETC B1_PR = ARM_UP;
int i = 0;
int z =0;

int inter_count = 0;


BOOL step = FALSE;
int index = 0;

sig_type flag_sig = CONTINUE;
int t = 0;

int mode = 0;

BOOL BOUNCE = FALSE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void go_strt(XY set);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char Rx_Buffer[BUFF_SIZE];
char Rx_indx = 0, Rx_data[2] = {0}, EndOfTrans = 0;
int Rx_Size = 0;



double angle[ANG_SIZE] = {0,30,60,90,120,150,180};
char Rx_Buffer_4[BUFF_SIZE]="0 0 0";
char RX_data[2] = {0};

//int mode = 1;
#define p(x) Rx_Buffer_4[++index] = x+0x30;
BOOL checkrange(XY endeffecter);
int x_t = 0 , y_t = 0;
BOOL flag2 = FALSE;

int step_count = 0;
int step_motor(int stp)
{
  step_count = (stp+8)%8;
  switch(step_count){
  case 0:
    WritePin(IN1,HIGH);
    WritePin(IN2,LOW);
    WritePin(IN3,HIGH);
    WritePin(IN4,LOW);
    break;
    
  case 1:
    WritePin(IN1,LOW);
    WritePin(IN2,LOW);
    WritePin(IN3,HIGH);
    WritePin(IN4,LOW);
    break;
    
  case 2:
    WritePin(IN1,LOW);
    WritePin(IN2,HIGH);
    WritePin(IN3,HIGH);
    WritePin(IN4,LOW);
    break;
    
  case 3:
    WritePin(IN1,LOW);
    WritePin(IN2,HIGH);
    WritePin(IN3,LOW);
    WritePin(IN4,LOW);
    break;
    
  case 4:
    WritePin(IN1,LOW);
    WritePin(IN2,HIGH);
    WritePin(IN3,LOW);
    WritePin(IN4,HIGH);
    break;
    
  case 5:
    WritePin(IN1,LOW);
    WritePin(IN2,LOW);
    WritePin(IN3,LOW);
    WritePin(IN4,HIGH);
    break;
    
  case 6:
    WritePin(IN1,HIGH);
    WritePin(IN2,LOW);
    WritePin(IN3,LOW);
    WritePin(IN4,HIGH);
    break;
    
  case 7:
    WritePin(IN1,HIGH);
    WritePin(IN2,LOW);
    WritePin(IN3,LOW);
    WritePin(IN4,LOW);
  }
  HAL_Delay(2-1);
  return step_count;
}

void TEST(void)
{
  sprintf(Rx_Buffer_4,"%d %d %d",x_t,y_t,inter_count);
  leng = strlen(Rx_Buffer_4);
#if PROGRAM == 2
  insert(&queue,Rx_Buffer_4);
#endif
  if(y_t == 100)
    flag2 =TRUE;
  if(y_t == 0)
    flag2 = FALSE;
  if(!flag2)
    x_t++, y_t++;
  else
    x_t--,y_t--;
#if PROGRAM == 1
  HAL_Delay(100);
#endif
  Ready_To_Rx = TRUE;
}


void Check()
{
  if( HAL_GPIO_ReadPin(ARM_BT_UP_GPIO_Port,ARM_BT_UP_Pin) == HIGH )
  {
    B1_PR = ARM_DOWN;
    WritePin(LED2,HIGH);
  }
  if( HAL_GPIO_ReadPin(ARM_BT_DW_GPIO_Port,ARM_BT_DW_Pin) == LOW )
  {
    B1_PR = ARM_UP;
    WritePin(LED2,LOW);
  }
}


void ReceiveData()
{
  goal.x = 0;
  goal.y = 0;
  
  if (is_empty(&queue) == NOT_EMPTY && Ready_To_Rx == TRUE)
  {
    
      strcpy(move_str, delete_front(&queue));
      len = strlen(move_str);
      
      int status = 1;
      goal.x = 0;
      goal.y = 0;
      
      z = 0;
      int format = 1;
      for (int i = len - 1; i >= 0; i--)
      {
        if (move_str[i] == ' ')
        {
          status++;
          i--;
          format = 1;
        }
        switch (status)
        {
        case 1://z
          k = move_str[i] - 0x30;
          z = k;
          //format *= 10;
          break;
        case 2://y
          if(move_str[i] == '-')
            goal.y = -goal.y;
          else{
            k = move_str[i] - 0x30;
            goal.y += (k)*format;
            format *= 10;}
          break;
        case 3://x
          if(move_str[i] == '-')
            goal.x = -goal.x;
          else{
            k = move_str[i] - 0x30;
            goal.x += (k)*format;
            format *= 10;}
        }
      }
//      goal = convert(goal);
//      position = xy_ang(goal.x, goal.y);
      
      if(z == 1)
      {
        goal = convert(goal);
        position = xy_ang(goal.x, goal.y);
        
        while(B1_PR == ARM_DOWN)//ÆÈ ¿Ã¸²
        {
          i = step_motor(--i);
        }
        if(flag_sig == START)
          for(int j = 700 ; j >= 0; j--){
            i = step_motor(--i);
          }
        
        flag_sig = START;
        
        position.seta1 = INIT_ANG + rad_deg(position.seta1);
        position.seta2 =  rad_deg(position.seta2);
        
        def_CCR1(position.seta1); //ÆÈÀÌµ¿
        def_CCR2(position.seta2);
        HAL_Delay(1000);
        
        while(B1_PR == ARM_UP)//ÆÈ ³»¸²
        {
          i = step_motor(++i);
          Check();
        }
        while(B1_PR == ARM_DOWN)//ÆÈ ¿Ã¸²
        {
          i = step_motor(--i);
          Check();
        }
      }
      else
      {
        if(z == 2)
        {
          while(t!=500) //ÆÈ ¿Ã¸²
          {
            i = step_motor(--i);
            t++;
          }
        }
        if(z == 0)
        {
          t = 0;
        }
        //move_line(goal);
        
        goal = convert(goal);
        position = xy_ang(goal.x, goal.y);
        
        position.seta1 = INIT_ANG + rad_deg(position.seta1);
        position.seta2 = /*-90.0 +*/ rad_deg(position.seta2);
        def_CCR1(position.seta1);
        def_CCR2(position.seta2);
        HAL_Delay(100-1);
        while(B1_PR == ARM_UP)//ÆÈ ³»¸²
        {
          i = step_motor(++i); 
          Check();
        }
        while(B1_PR == ARM_DOWN)//ÆÈ ¿Ã¸²
        {
          i = step_motor(--i);
          Check();
        }      
        for(int j = 0 ; j < 45 ; j++)//ÆÈ ¿Ã¸² ÆæÀÇ ¸¶ÂûÀÌ ½ÉÇÒ¼ö·Ï ½ºÅÜ Ãß°¡
        {
          i = step_motor(--i);
        }
      }
      goal.x = 0; goal.y = 0;
      leng = 0;
  }
  else if(is_empty(&queue) == EMPTY && Ready_To_Rx == TRUE)
  {
    Ready_To_Rx = FALSE;
    flag_sig = STOP;
    init_dqueue(&queue);
  }
}


void ReceiveData_1()
{
  goal.x = 0;
  goal.y = 0;
  
  if (is_empty(&queue) == NOT_EMPTY && Ready_To_Rx == TRUE)
  {
    
      strcpy(move_str, delete_front(&queue));
      len = strlen(move_str);
      
      int status = 1;
      goal.x = 0;
      goal.y = 0;
      
      z = 0;
      int format = 1;
      for (int i = len - 1; i >= 0; i--)
      {
        if (move_str[i] == ' ')
        {
          status++;
          i--;
          format = 1;
        }
        switch (status)
        {
        case 1://z
          k = move_str[i] - 0x30;
          z = k;
          //format *= 10;
          break;
        case 2://y
          if(move_str[i] == '-')
            goal.y = -goal.y;
          else{
            k = move_str[i] - 0x30;
            goal.y += (k)*format;
            format *= 10;}
          break;
        case 3://x
          if(move_str[i] == '-')
            goal.x = -goal.x;
          else{
            k = move_str[i] - 0x30;
            goal.x += (k)*format;
            format *= 10;}
        }
      }
//      goal = convert(goal);
//      position = xy_ang(goal.x, goal.y);
      
      if(z == 1)
      {
        //**
        before = goal;
        //**
        
        goal = convert(goal);
        position = xy_ang(goal.x, goal.y);
        
        while(B1_PR == ARM_DOWN)//ÆÈ ¿Ã¸²
        {
          i = step_motor(--i);
        }
        if(flag_sig == START)
          for(int j = 700 ; j >= 0; j--){
            i = step_motor(--i);
          }
        
        flag_sig = START;
        
        position.seta1 = INIT_ANG + rad_deg(position.seta1);
        position.seta2 =  rad_deg(position.seta2);
        
        def_CCR1(position.seta1); //ÆÈÀÌµ¿
        def_CCR2(position.seta2);
        HAL_Delay(1000);
        
        while(B1_PR == ARM_UP)//ÆÈ ³»¸²
        {
          i = step_motor(++i);
          Check();
        }
        while(B1_PR == ARM_DOWN)//ÆÈ ¿Ã¸²
        {
          i = step_motor(--i);
          Check();
        }
      }
      else
      {
        if(z == 2)
        {
          while(t!=500) //ÆÈ ¿Ã¸²
          {
            i = step_motor(--i);
            t++;
          }
        }
        if(z == 0)
        {
          t = 0;
        }
        /*
              ÀÌ°÷¿¡ Á÷¼±À¸·Î ÀÌ¾îÁÖ´Â ÇÔ¼ö Ãß°¡
        */
        
       draw_line(&before,&goal);
       before = goal;
        
        //////////////////////////////////////////////////////
//        goal = convert(goal);
//        position = xy_ang(goal.x, goal.y);
//        
//        position.seta1 = INIT_ANG + rad_deg(position.seta1);
//        position.seta2 = /*-90.0 +*/ rad_deg(position.seta2);
//        def_CCR1(position.seta1);
//        def_CCR2(position.seta2);
        HAL_Delay(100-1);
        while(B1_PR == ARM_UP)//ÆÈ ³»¸²
        {
          i = step_motor(++i); 
          Check();
        }
        while(B1_PR == ARM_DOWN)//ÆÈ ¿Ã¸²
        {
          i = step_motor(--i);
          Check();
        }      
        for(int j = 0 ; j < 45 ; j++)//ÆÈ ¿Ã¸² ÆæÀÇ ¸¶ÂûÀÌ ½ÉÇÒ¼ö·Ï ½ºÅÜ Ãß°¡
        {
          i = step_motor(--i);
        }
      }
      goal.x = 0; goal.y = 0;
      leng = 0;
  }
  else if(is_empty(&queue) == EMPTY && Ready_To_Rx == TRUE)
  {
    Ready_To_Rx = FALSE;
    flag_sig = STOP;
    init_dqueue(&queue);
  }
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  
  
  init_dqueue(&queue);
  
  volatile int a = 0;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  
 
 def_CCR1(0); def_CCR2(0);
    
  goal = convert(goal);
  position = xy_ang(goal.x, goal.y);
  
  position.seta1 = INIT_ANG + rad_deg(position.seta1);
  position.seta2 = rad_deg(position.seta2);
  

  def_CCR1(position.seta1);
  def_CCR2(position.seta2);
  
  HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_Buffer,1);
  HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_Buffer,1);
  HAL_TIM_Base_Start_IT(&htim4);
  
  B1_PR = ARM_UP;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if PROGRAM == 1
    ReceiveData_2();
#elif PROGRAM == 2
    if(flag_sig == STOP)
    {
      while(mode != 800) //ÆÈ ¿Ã¸²
      {
        i = step_motor(--i);
        mode++;
      }  
      mode = 0;
      goal.x = goal.y = 0.0;
      goal = convert(goal);
      position = xy_ang(goal.x, goal.y);
      
      position.seta1 = INIT_ANG + rad_deg(position.seta1);
      position.seta2 = rad_deg(position.seta2);
      
      def_CCR1(position.seta1);
      def_CCR2(position.seta2);
      HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_Buffer,1);
      HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_Buffer,1);
      flag_sig = CONTINUE;
    }
    ReceiveData();
    //    def_CCR1(0); def_CCR2(0);
#endif
  
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 840-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, IN4_Pin|ENB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENA_Pin */
  GPIO_InitStruct.Pin = ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ENA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN1_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin ENB_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|ENB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : ARM_BT_DW_Pin */
  GPIO_InitStruct.Pin = ARM_BT_DW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ARM_BT_DW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARM_BT_UP_Pin */
  GPIO_InitStruct.Pin = ARM_BT_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ARM_BT_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN3_Pin */
  GPIO_InitStruct.Pin = IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IN3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
//_weak ÇÔ¼öÀçÁ¤ÀÇ:¿À¹ö¶óÀÌµù
int cur_time = 0 , pre_time = 0;
int diff_time = 0, total_time = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
  if(GPIO_Pin == B1_Pin)
  {
    if(B1_PR == ARM_UP)
    {
      B1_PR = ARM_DOWN;
      WritePin(LED2,HIGH);
    }
    else if(B1_PR == ARM_DOWN)
    {
      B1_PR = ARM_UP;
      WritePin(LED2,LOW);
    }
  }
  if(GPIO_Pin == ARM_BT_UP_Pin)
  {
    if(B1_PR == ARM_DOWN)
    {
      B1_PR = ARM_UP;
      WritePin(LED2,LOW);
    }
    for(volatile int i = 0 ; i <= 1000 ; i++)
      Check();
  }
  if(GPIO_Pin == ARM_BT_DW_Pin)
  {
    if(B1_PR == ARM_UP)
    {
      B1_PR = ARM_DOWN;
      WritePin(LED2,HIGH);
    }
    for(volatile int i = 0 ; i <= 1000 ; i++)
      Check();
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4)
    Check();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  if(huart->Instance == USART1)
  {
    Rx_Buffer_4[index++] = Rx_Buffer[0];
    
    if(step == FALSE && index == 2)
    {
      leng = 0;
      step = TRUE; index = 0;
      leng += (Rx_Buffer_4[0] - 0x30)*10;
      leng += (Rx_Buffer_4[1] - 0x30);
    }
    if(step == TRUE && index == leng)
    {
      step = FALSE; 
      
      Rx_Buffer_4[index] = '\0';
      index = 0;
      
      
      if(strcmp(Rx_Buffer_4,"1") == 0)
      {
        Ready_To_Rx = TRUE;
        
        HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_Buffer,1);
        // flag_sig = START;
        return;
      }
      
      insert(&queue,Rx_Buffer_4);
    }
    
    HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_Buffer,1);
  }
  
  if(huart->Instance == USART2)
  {
    Rx_Buffer_4[index++] = Rx_Buffer[0];
    
    if(step == FALSE && index == 2)
    {
      leng = 0;
      step = TRUE; index = 0;
      leng += (Rx_Buffer_4[0] - 0x30)*10;
      leng += (Rx_Buffer_4[1] - 0x30);
    }
    if(step == TRUE && index == leng)
    {
      step = FALSE; 
      
      Rx_Buffer_4[index] = '\0';
      index = 0;
      
      
      if(strcmp(Rx_Buffer_4,"1") == 0)
      {
        Ready_To_Rx = TRUE;
        
        HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_Buffer,1);
        // flag_sig = START;
        return;
      }
      
      insert(&queue,Rx_Buffer_4);
    }
    
    HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_Buffer,1);
  }
}

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
