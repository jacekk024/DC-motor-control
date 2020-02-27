/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author			: Kotra & Piskorz Electronics
  * @date			: 20.02.2020
  * @brief          : STM32 program for Direct Motor Speed control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math_helper.h"
#include "i2c-lcd.h"
#include <math.h>
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

float value_f;/**<Wartosc pobierana przez ADC>*/
uint8_t Received[3];/**<Tablica przechowujaca wiadomosc z UARTA>*/
int pwm_duty;/**<Wartosc wypelnienia sterujaca predkosc silnika>*/
int32_t current_speed;/**<Zmienna przechowujaca akutalna predkosc>*/
int32_t wanted_speed;/**<Zmienna przechowujaca zadana predkosc>*/
int32_t pid_error;/**<Zmienna obliczanego uchybu>*/




//*filtr*//
uint32_t adcValue[1];/**<Zmienna przechowujaca wartosc zczytana z ADC>*/
uint32_t adcValue1[1];/**<Zmienna przechowujaca wartosc po odfiltrowaniu>*/
float32_t signal_in = 0;/**<Zmienna przechowujaca wartosc wejsciowa do FIR>*/
float32_t signal_out = 0;/**<Zmienna przechowujaca wartosc wyjsciowa z FIR>*/
//***********************//
#define TEST_LENGTH_SAMPLES  1
#define BLOCK_SIZE            1
#define NUM_TAPS              29
#define PID_PARAM_KP	120
#define PID_PARAM_KI	49
#define PID_PARAM_KD	0.7


arm_fir_instance_f32 S;/**<Obiekt przechowujacy FIR>*/
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
static float32_t firCoeffs[NUM_TAPS] = {-0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f,
    +0.0085302217f, -0.0000000000f, -0.0173976984f, -0.0341458607f, -0.0333591565f,
    +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f,
    +0.2229246956f, +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f,
    -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f, +0.0080754303f,
    +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f};

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


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
	arm_pid_instance_f32 PID;/**<Zmienna przechowujaca obiekt PID>*/
	PID.Kp = PID_PARAM_KP;/**<Zmienna przechowujaca wartosc KP>*/
	PID.Ki = PID_PARAM_KI;/**<Zmienna przechowujaca wartosc KI>*/
	PID.Kd = PID_PARAM_KD;/**<Zmienna przechowujaca wartosc KD>*/
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /**
   * inicjalizowanie obiektu PID
   */
  	arm_pid_init_f32(&PID, 1);
  /**
    * inicjalizowanie obiektu LCD
    */
  	lcd_init ();
  	/**
  	  * inicjalizowanie podstawowych funkcji mikroporcesora
  	  */
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_ADC_Start(&hadc1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_UART_Receive_IT(&huart3,Received,3);
	/**
	  * inicjalizowanie obiektu FIR
	  */
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs[0], (float32_t *)&firStateF32[0], blockSize);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 /**
	   *
	   * @brief Funkcja glowna programu.Odpowiada za odbieranie warotsci z ADC, uzycie filtra FIR oraz regulatora PID oraz zadaniu wypelniena PWM.
	   *
	   * */
  while (1)
  {


	  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {

	  	 value_f = HAL_ADC_GetValue(&hadc1);
	  	 arm_fir_f32(&S, &value_f, &signal_out, blockSize);
	  	 current_speed =  (int32_t)(signal_out/2.4);
	  	 HAL_ADC_Start(&hadc1);
	  	  }
	  arm_fir_f32(&S, &value_f, &signal_out, blockSize);
	  current_speed =  (int32_t)(signal_out/2.4);
	   pid_error = wanted_speed-current_speed;
	   pwm_duty = (int32_t)arm_pid_f32(&PID, pid_error);

	  	 if(pwm_duty >= 1000)
	  	  {
	  		  pwm_duty /= 10000;
	  	  }
	  	  else if(pwm_duty <= 300)
	  	  	  {
	  		  	  pwm_duty = 300;
	  	  	  }
	  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_duty);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(1); // Main loop delay
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief	Odpowiada za utworzenie przerwania przy przesylaniu danych przez UART
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		  	wanted_speed = (int32_t)(atoi(Received)*7 + 300);
		  	 HAL_UART_Receive_IT(&huart3,&Received,3);
}
/**
 * @brief	Przerwanie timera odpowiadajacego za wyswietlenie warotsci na wyswietlaczu LCD
 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
	char speed[4];/**<Tablica przechowujaca wartosc obecnej predkosci>*/
	char speed2[4];/**<Tablica przchowujaca wartosc predkosci zadanej>*/
	sprintf(speed,"%i",current_speed);
	sprintf(speed2,"%i",wanted_speed);

	if(htim->Instance == TIM4)
	{
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("current:");
		lcd_send_string(speed);
		lcd_put_cur(1, 0);

		lcd_send_string("wannted:");
		lcd_send_string(speed2);
		lcd_put_cur(0, 0);





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

  //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  while(1)
  {
	  //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  //HAL_Delay(100);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
