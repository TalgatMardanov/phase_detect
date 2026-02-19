/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/**
тайминги работы синхронизации с частотой сети
**/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t t=0;//тестовая переменная проверки канала EXTI11 PORT PE11 кнопка S2 на макетки
uint16_t timer=0;//контейнер переменная для подсчета пройденного времени 1000 ms
uint16_t timer_sck=0;//контейнер переменная проденного времени за 10 ms
uint16_t hz=0;//контейнер переменная для аодсчета положительных фронтов сигнала по частоте
uint16_t hz_valid=0;//переменная в которой будет отображаться частота 
uint32_t timer3=0;//контейнер переменная в таймере TIM3 с заполнением (APB2?)84Мгц/(840-1)(заполнение)=100 000 подсчетов за 1 секунду
uint32_t timer3_valid=0;
uint16_t p=1;//переменная смещения от положительного фронта частоты
uint8_t count=0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  timer++;//подсчитывает сколько ms уже прошло с включения
  timer_sck++;//подсчитывает сколько ms уже прошло с включения но нужен был для определения
  
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  count++;
  if (count >=2){count=0;}
  timer3 = 0;
  //главное, как только положительный фронт наступает TIM3 сбрасываю на 0, ЭТО ОЧЕНЬ ВАЖНО!!!
  hz++;//тут подсчитывает сколько положительных фронтов
  if (timer >= 1000) {//как только таймер подсчета достик 1000 ms = 1 sec
  hz_valid=hz;//контейнер подсчета переменной hz передает переменной hz_valid
  timer=0;//сбрасываем счетчик для нового раунда подсчета времени
  hz=0;//сбрасываем счетчик для нового раунда подсчета за 1000 ms контейнера hz положительных фронтов
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);//тестовая индикация что процесс работает, 1000 ms меняет состояние
  }
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  timer3++;//контейнер начинает считать по TIM3 с частотой 84 000 000 Гц, до заполнения величины 849 и только тогда прибавляет счёт!!!
  //p=200;//смещение от положительного фронта не должно превышать 1000-10 единиц!!!
  if (timer3 >= p & timer3 <= p+50){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);}  //вот главный пидор, ебался я с ним долго
  //if (timer3 >= p & count == 0){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);} 
  //if (timer3 >= 100 & count == 1){HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);} 
  //if (timer3 >= p) {HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);}
  //Объясняю, как только timer3 заполнится до переменной P и меньше переменной P+10 порт B7 уходит в положительный фронт, если нет то в отрицательный ОБЯЗАТЕЛЬНО ELSE!!!!!
  //и суко это данную строку надо вставлять ИМЕННО в ЭТУ функцию!!!!!где происходит быстрее всего сравнивание!!!
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  t++;//подсчет тестовой кнопки
  p=p+50;
  if (p >= 800) {p=100;}
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
