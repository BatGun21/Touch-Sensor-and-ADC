/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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

/* USER CODE BEGIN PV */
uint32_t acqVal = 0;
char acqStr[11];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Tx(char c);
char UART_Rx(void);
void UART_Send(char str[]);
void UART_Receive(char str[]);
void reverseString(char str[], int length);
void intToString(uint32_t value, char str[]);
void str_empty (char str[]);
void SysTick_Init(uint32_t ticks);
void DelayMS(unsigned int time);
void LED_Init(void);
void UART1_Init(void);
int adc_check (void);
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
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Init(8000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  LED_Init();
  UART1_Init();

  /*------ INIT for TSC -----*/
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enabling Clock for Port B

  RCC->AHBENR |= RCC_AHBENR_TSCEN; //Enabling Clock for Touch Sensor

  GPIOB->AFR[0] |= 0x00000033; //Enabling the Alternate Functions for PB0 (Sampling) and PB1 (Channel)
  GPIOB->MODER |= 0x0000000a; // Setting to Alternate Mode
  GPIOB->OTYPER |= 0x00000001; // Setting PB0 as Sampling (Open Drain) and PB1 as Channel {output Push Pull}
  GPIOB->OSPEEDR |= 0x00000003; // Setting as high speed

  // Configuring TSC

  TSC->CR |= (TSC_CR_PGPSC_2 | TSC_CR_PGPSC_0); // Setting the Pulse Generator frequency as fclk/32
  TSC->CR |=  (TSC_CR_CTPH_0 | TSC_CR_CTPL_0); // Setting the duration of high and low state as 2x tPGCLK
  TSC->CR |= (TSC_CR_MCV_2 | TSC_CR_MCV_1); // Max Count Value is 16383


  TSC->IOHCR &= (uint32_t)(~(TSC_IOHCR_G3_IO2 | TSC_IOHCR_G3_IO3)); // Disabling the Hysteresis
  TSC->IOASCR |= 0x600; // Enabling the analog switch for G3_IO2 and G3_IO3
  TSC->IER |= TSC_IER_EOAIE; // Enabling End of Acquistion Interrupt
  TSC->IOSCR |= TSC_IOSCR_G3_IO2; // Sampling enabled for G3IO2
  TSC->IOCCR |= TSC_IOCCR_G3_IO3; // Channel Enabled for G3IO3
  TSC->IOGCSR |= TSC_IOGCSR_G3E; // Enable Group 3

  TSC->CR |= TSC_CR_TSCE; // Enabling TSC


  /*----- ADC INIT -----*/

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enabling Clock for GPIOA
  GPIOA->MODER |= 0x0c; // Setting PA1(ADC_IN1) in analog input mode.


  RCC->APB2ENR |= 0x200; // Enabling Clock for ADC
  RCC->CR2 |= RCC_CR2_HSI14ON; // Turning on dedicated 14MHz Clock for ADC
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0)
  {
	  __NOP();
  }
  ADC1->CFGR2 &= (~ADC_CFGR2_CKMODE); // Choosing the Aschronous CLk


  ADC1->CFGR1 |= ADC_CFGR1_CONT; // Set ADC to continuous mode

  ADC1->CHSELR |= 0x02; // Select the ADC channel (PA1)

  ADC1->ISR |= ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  while ((ADC1->ISR & ADC_ISR_ADRDY) != 0x01)
  {
	  __NOP();
  }

  ADC1->CR |= ADC_CR_ADSTART; // Start ADC conversion

  /*  USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  TSC->CR |= TSC_CR_START; // Start the Touch Sensing
//	  if ((TSC->ISR & TSC_ISR_EOAF) == TSC_ISR_EOAF){
//		  acqVal = TSC->IOGXCR[2];  // Get G3 counter value
//		  TSC->ICR = TSC_ICR_EOAIC; // Clear EOAF
//	  }
//	  intToString(acqVal, acqStr);
//	  UART_Send(acqStr);
//	  str_empty(acqStr);
//	  DelayMS(1000);

	  if (adc_check()){

	  }



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int adc_check (void){
	int check = 0;
	if ((ADC1->ISR & ADC_ISR_EOC == ADC_ISR_EOC)){
		check = 1;
	}
	return check;
}


void intToString(uint32_t value, char str[]) {
    int i = 0;

    if (value == 0) {
        str[i++] = '0';
    } else {
        while (value) {
            str[i++] = '0' + (value % 10);  // Convert digit to character
            value /= 10;
        }
    }

    str[i] = '\0';  // Null-terminate the string
    reverseString(str, i);
}

void reverseString(char str[], int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

void str_empty (char str[]){
	int i = 0;
	while (str[i] != '\0'){
		str[i] = '\0';
		i++;
	}
}

void UART_Tx(char c){

	while ((USART1->ISR & USART_ISR_TXE) != USART_ISR_TXE){
		__NOP();
	}
	USART1->TDR = c;
}

char UART_Rx(void){

	char c;

	while ((USART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE){
		__NOP();
	}
	c = USART1->RDR;

	return c;
}

void UART_Send(char str[]){
	int i = 0;
	while (str[i] != '\0'){
		UART_Tx(str[i]);
		i++;
	}
}

void UART_Receive(char str[]) {

	int counter = 0;
	int signal = 0;
	while (signal == 0){
		str[counter] = UART_Rx();
		if ((str[counter] == '\n') || (str[counter] == '\r') || str[counter] == '\0'){
			counter = 0;
			signal = 1;
		}else{
			counter++;
		}
	}
}

void DelayMS(unsigned int time){

	for(int i=0; i<=time; i++){
		while ((SysTick->CTRL & 0x00010000) == 0){
				//Wait for 1 millisec.
		}
	}
}

void SysTick_Init(uint32_t ticks){

	SysTick->CTRL = 0; // Disable SysTick

	SysTick->LOAD = ticks-1; // Set Reload Register

	// Setting Interrupt Priority to the highest
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);

	SysTick->VAL = 0; // Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Selecting internal clock source
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enabling SysTick exception Request when 0


	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick
}

void LED_Init(void){
	/*------ GPIOC INIT for LEDS -----*/

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enabling Clock for Port C

	GPIOC->MODER |= GPIO_MODER_MODER6_0; //Set bit 0 to 1 Red
	GPIOC->MODER |= GPIO_MODER_MODER7_0; //Set bit 0 to 1 Blue
	GPIOC->MODER |= GPIO_MODER_MODER8_0; //Set bit 0 to 1 Orange
	GPIOC->MODER |= GPIO_MODER_MODER9_0; //Set bit 0 to 1 Green
}

void UART1_Init(void){
	/* ----- USART1 INIT ----- */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 	 // Enabling Clock for Port A

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enabling Clock for USART1

	GPIOA->OSPEEDR |= 0x003c0000; // Setting the speed of the PA9 and PA10 to High Speed
	GPIOA->AFR[1] |= 0x00000110; // PA9 is TX and PA10 is RX
	GPIOA->MODER |= 0x00280000; // Setting up Pins PA9 and PA10 for Alternate Function

	USART1->BRR = 0x0341; // Setting up Baud Rate to 9600 bps, Oversampling by 16;

	USART1->CR1 |= USART_CR1_TE ; // Enabling UART Transmit
	USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE; // Enabling UART Receive
	USART1->CR1 |= USART_CR1_UE; // Enabling UART
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
