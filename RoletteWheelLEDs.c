/** * ******************************************************************************
  * File Name     : Main.c
  * PROJECT       : PROG8125 -Assignment #4 Roulette Wheel
  * PROGRAMMER    : Abdelraheem Alkuor (Abdel)
  * FIRST VERSION : 2016-11-27
  * Description   : This program is a demostration of using a state-machine.
  * 				Roulette LED wheel of 8 LEDS are used where LEDs are turned
  * 				on and off in sequential(Current LED is on previous LED is off)
  * 				and "click" sound is produced when and LED is turned on.
  * 				The roulette LED wheel has three states as follows:
  *					1) When user pushbutton is not pressed(normal mode) LEDs spinning
  *					   speed is in normal speed (200ms) between LED off and on.
  *					2) When user pushbutton is pressed and kept pressed(fast mode)
  *					   LEDs spinning speed between on and off is fast (50ms)in our
  *					   case.
  *					3) When user pushbutton is released (slow mode) LEDs spinning
  *					   speed is slowed down incrementally by 200ms until it reaches
  *					   2000ms then it stops completely. The last LED where it stopped
  *					   at will blink 4 times and stays on for 5 seconds then to goes
  *					   back to the first state(normal mode) and so on.
  *
  *					Note: "Click" sound effect will be produced according to the current state
  *						   of roulette LED wheel. The sound speed is coherent with
  *						   the speed of roulette LED wheel.
  *
  *
  ******************************************************************************
  *****************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "roulette_sound_data.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// states used for roulette wheel example
#define WAITING_FOR_PUSHBUTTON_PRESS_STATE		0
#define WAITING_FOR_PUSHBUTTON_RELEASE_STATE	1
#define WAITING_FOR_LED_PATTERN_COMPLETE_STATE	2

// led flash pattern control constants
#define INITIAL_LED_DELAY						200
#define BUTTON_PRESSED_LED_DELAY				50
#define SLOW_DOWN_CONSTANT						150
#define STOP_SPINNING_VALUE						2000
#define WAIT_FOR_NEXT_ROUND_DELAY				5000
#define DONE_BLINK_RATE_DELAY					500
#define NUMBER_OF_FLASHES_WHEN_DONE				5

// delayAfterTurnOn values
#define DO_NOT_PUT_A_DELAY_AFTER_TURN_ON		0
#define PUT_A_DELAY_AFTER_TURN_ON				1

// checkPushButtonDebounced return values
#define BUTTON_PRESSED							1
#define BUTTON_NOT_PRESSED						0

// debounce delay time in ms
#define DEBOUNCE_DELAY							2
#define MILLI_SECOND_FACTOR						1000
#define MICROSECOND_BETWEEN_SAMPLES				62
// an array holding the pattern of LEDs
#define LED_PATTERN_SIZE	8
static uint16_t ledPattern[]={LD4_Pin, LD3_Pin, LD5_Pin, LD7_Pin, LD9_Pin, LD10_Pin, LD8_Pin, LD6_Pin};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
void turnOnLed( uint16_t ledNumber, int delayAfterTurnOnFlag);
int checkPushButtonDebounced( void );
void delayRoletteSound(__IO uint32_t Delay);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// FUNCTION      : turnOnLed
// DESCRIPTION   :
//   This function will turn on a given LED using supplied GPIO pin number and turn off the last one
// PARAMETERS    :
//   ledNumber   : LED pin to turn on
//   delayAfterTurnOnFlag : if set put a delay after the led is turn on in
//
// RETURNS       :
//   Nothing
void turnOnLed( uint16_t ledNumber, int delayAfterTurnOnFlag )
{
	static uint16_t lastLed = 0xffff;	// not set yet so set the value to invalid one
	if( lastLed != 0xffff)
	{
		// clear the last LED if needed
		HAL_GPIO_WritePin(GPIOE, lastLed, GPIO_PIN_RESET );
	}
	// keep track of last led that we turned on so we can turn it off next time
	lastLed = ledNumber;
	if( delayAfterTurnOnFlag == PUT_A_DELAY_AFTER_TURN_ON )
	{
		delayRoletteSound( 500 );
	}
	// turn on the LED pin on GPIO
	HAL_GPIO_WritePin(GPIOE, lastLed, GPIO_PIN_SET );
}

// FUNCTION      : checkPushButtonDebounced
// DESCRIPTION   :
//   This function will return the state of the push button on the STM debounced
// PARAMETERS    :
//   None
// RETURNS       :
//   0 if not pushed else non zero
int checkPushButtonDebounced( void )
{
	int buttonState = BUTTON_NOT_PRESSED;
	// get the initial button state
	if( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin ) != 0 )
	{
		buttonState = BUTTON_PRESSED;
	}
	delayRoletteSound( DEBOUNCE_DELAY );// small delay to bypass bounce on push button

	// check if the button state does not agree with first sample
	// if it does not agree ignore it
	if( buttonState == BUTTON_PRESSED )
	{
		if( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin ) == 0 )
			buttonState = BUTTON_NOT_PRESSED;
	}
	else
	{
		if( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin ) != 0 )
			buttonState = BUTTON_PRESSED;
	}

	return buttonState;
}

/* USER CODE END 0 */

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DAC_Init();
	HAL_DAC_Start( &hdac, DAC_CHANNEL_1);


	uint8_t ledIndex = 0;// used to select LED to turn on, initialize LED roulette starting point
	uint32_t delayTime = INITIAL_LED_DELAY;
	uint8_t stateMachine = WAITING_FOR_PUSHBUTTON_PRESS_STATE;

	/* USER CODE BEGIN WHILE */
	while (1)
	{
		switch (stateMachine)
		{
			case WAITING_FOR_PUSHBUTTON_PRESS_STATE:
				delayRoletteSound(delayTime);
				turnOnLed(ledPattern[ledIndex++], DO_NOT_PUT_A_DELAY_AFTER_TURN_ON);// turn on the LED (and turn off the old one)
				if (ledIndex >= LED_PATTERN_SIZE)// is the pattern ready to repeat
				{
					ledIndex = 0;
				}
				if (checkPushButtonDebounced() == BUTTON_PRESSED)
				{
					stateMachine = WAITING_FOR_PUSHBUTTON_RELEASE_STATE;
				}
				break;
			case WAITING_FOR_PUSHBUTTON_RELEASE_STATE:
				while (checkPushButtonDebounced() != BUTTON_NOT_PRESSED)
				{
					turnOnLed(ledPattern[ledIndex++],DO_NOT_PUT_A_DELAY_AFTER_TURN_ON);
					if (ledIndex >= LED_PATTERN_SIZE)
					{
						ledIndex = 0;
					}
					delayRoletteSound(BUTTON_PRESSED_LED_DELAY);
				}
				stateMachine = WAITING_FOR_LED_PATTERN_COMPLETE_STATE;
				break;
			case WAITING_FOR_LED_PATTERN_COMPLETE_STATE:
			default:
				//Slow down roulette leds spinning by 200ms incremental
				for (int i = SLOW_DOWN_CONSTANT; i < STOP_SPINNING_VALUE; i +=SLOW_DOWN_CONSTANT)
				{
					turnOnLed(ledPattern[ledIndex++],DO_NOT_PUT_A_DELAY_AFTER_TURN_ON);
					if (ledIndex >= LED_PATTERN_SIZE)
					{
						ledIndex = 0;
					}
					delayRoletteSound(i);
				}
				//blink the last LED where roulette led wheel stopped at
				for (int i = 0; i < NUMBER_OF_FLASHES_WHEN_DONE; i++)
				{
					turnOnLed(ledPattern[ledIndex - 1], PUT_A_DELAY_AFTER_TURN_ON);
					delayRoletteSound(DONE_BLINK_RATE_DELAY);
				}
				delayRoletteSound(WAIT_FOR_NEXT_ROUND_DELAY);
				stateMachine = WAITING_FOR_PUSHBUTTON_PRESS_STATE;
				break;
		}

	}
  /* USER CODE END 3 */

}
/*
 * FUNCTION      : delayRoletteSound
 * DESCRIPTION   : This function is used to produce delay in millisecond
 * 				   and it produces an output "Tick" sound on PA04 using
 * 				   DAC. The sound speed is consistent with the required
 * 				   delay i.e delay is short the output sound will be fast
 * PARAMETERS    :Delay: In millisecond
 *Return		 :None
*/
void delayRoletteSound(__IO uint32_t Delay)
{
	uint32_t tickstart = HAL_GetTick();
	uint16_t i=0;
	while((HAL_GetTick() - tickstart) < Delay*MILLI_SECOND_FACTOR)
	{
		if((HAL_GetTick() - tickstart)%MICROSECOND_BETWEEN_SAMPLES==0)
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, rouletteSoundSamples[i++]);
			if(i==ROULETTE_SOUND_SIZE)// reset index to 0
			{
				i=0;
			}

		}
	}

}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PA11   ------> USB_DM
     PA12   ------> USB_DP
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

}
/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
