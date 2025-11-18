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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t MAX_DISTANCE_FROM_PROBE_CM;
uint16_t MIN_DISTANCE_FROM_PROBE_CM;
uint8_t  LoRaBufRX;
uint8_t  LoRaBufLongRX[5000];
uint16_t LoRaBufPosRX;

uint8_t turnOffUART_flag = 0;

GPIO_InitTypeDef GPIO_BufStruct = {0};

volatile uint16_t __attribute__((section(".ram2"))) measurementsInterval = 19;
volatile uint32_t __attribute__((section(".ram2"))) isAfterResetFlag=1;

/*typedef struct {
	uint32_t testRAM2_val;
	uint32_t ARR[100];
}sram2_located_data;

sram2_located_data* sram2_data = (sram2_located_data*)SRAM2_BASE;	*/

char LORA_TEST_Conf[] = "AT+TEST=RFCFG,869.525,SF12,125,12,15,14,ON,OFF,OFF\r\n";//869.525
char LORA_TEST_TransmitContinuousLoRaSignal[] = "AT+TEST= TXCLORA\r\n";
char LORA_TEST_SendHex[] = "AT+TEST=TXLRPKT, ";
char LORA_TEST_SendString[] = "AT+TEST=TXLRSTR, \"LoRaWAN Modem\"\r\n";
char LORA_TEST_RX[] = "AT+TEST=RXLRPKT\r\n";
char LORA_RSSI_RSSI[] = "AT+TEST = RSSI, 869.525, 500\r\n";
char LORA_LOWPOWER[] = "AT+LOWPOWER\r\n";
char LORA_LOWPOWER_EXTR_ON[] = "AT+LOWPOWER=AUTOON\r\n";
char LORA_LOWPOWER_EXTR_OFF[] = {0xFF, 0xFF, 0xFF, 0xFF, 'A','T','+','L','O','W','P','O','W','E','R',0x3D,'A','U','T','O','O','F','F','\r','\n'};
char LORA_TEST_STOP[] = "AT+TEST=STOP\r\n";
char LORA_TEST_START[] = "AT+TEST=START\r\n";
char LORA_RESET[] = "AT+RESET\r\n";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t MEAS_meas_cm()
{
	HAL_GPIO_WritePin(MEAS_TRIG_GPIO_Port, MEAS_TRIG_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start(&htim6);
	while(htim6.Instance->CNT < 10){} //MEAS_TRIG_Pin is HIGH for 10 us
	HAL_TIM_Base_Stop(&htim6);
	htim6.Instance->CNT = 0;
	HAL_GPIO_WritePin(MEAS_TRIG_GPIO_Port, MEAS_TRIG_Pin, GPIO_PIN_RESET); //MEAS_TRIG_Pin is LOW
	while(HAL_GPIO_ReadPin(MEAS_ECHO_GPIO_Port, MEAS_ECHO_Pin) == GPIO_PIN_RESET){} //Wait for ECHO_pin LOW
	HAL_TIM_Base_Start(&htim6);
	while(HAL_GPIO_ReadPin(MEAS_ECHO_GPIO_Port, MEAS_ECHO_Pin) == GPIO_PIN_SET){} //Wait for ECHO_pin is HIGH
	uint16_t measTime_us = htim6.Instance->CNT; //get time value
	HAL_TIM_Base_Stop(&htim6);
	htim6.Instance->CNT = 0;
	uint16_t distance_cm = measTime_us / 58; //count distance
	return distance_cm;
}

float MEAS_calculateAsPercent(uint16_t cmVal)
{
	return 100*((MIN_DISTANCE_FROM_PROBE_CM - MAX_DISTANCE_FROM_PROBE_CM) - (cmVal - MAX_DISTANCE_FROM_PROBE_CM))/(MIN_DISTANCE_FROM_PROBE_CM - MAX_DISTANCE_FROM_PROBE_CM);
}

void MEAS_measMaxDistance()
{
	MAX_DISTANCE_FROM_PROBE_CM = MEAS_meas_cm();
}

void MEAS_measMinDistance()
{
	MIN_DISTANCE_FROM_PROBE_CM = MEAS_meas_cm();
}

void MEAS_setMaxDistance(uint16_t maxVal)
{
	MAX_DISTANCE_FROM_PROBE_CM = maxVal;
}

void MEAS_setMinDistance(uint16_t minVal)
{
	MIN_DISTANCE_FROM_PROBE_CM = minVal;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  GPIO_BufStruct.Mode = GPIO_MODE_INPUT;
  GPIO_BufStruct.Pull = GPIO_PULLDOWN;

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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t resToSent;
  MIN_DISTANCE_FROM_PROBE_CM = 20;
  MAX_DISTANCE_FROM_PROBE_CM = 300;
  PWR->CR3 |= PWR_CR3_RRS;
  uint8_t buttonPressed;
  HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart1, &LoRaBufRX, 1);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)"A", 1); LORA_LOWPOWER_EXTR_OFF[]
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)LORA_LOWPOWER_EXTR_OFF, sizeof(LORA_LOWPOWER_EXTR_OFF));
  HAL_Delay(500);
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)LORA_TEST_START, strlen(LORA_TEST_START));
  HAL_Delay(500);
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)"AT+MODE=TEST\r\n", 14);
  HAL_Delay(500);
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)"AT+TEST=?\r\n", 11);
  HAL_Delay(500);
  HAL_UART_Transmit_IT(&huart1, LORA_TEST_Conf, strlen(LORA_TEST_Conf));
  //HAL_Delay(1000);
  //HAL_UART_Transmit_IT(&huart1, "AT+TEST=RXLRPKT\r\n", 17);
  HAL_Delay(1000);
  while (1)
  {
	  isAfterResetFlag = 1;
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	  HAL_GPIO_WritePin(TRANSISTOR_GPIO_Port, TRANSISTOR_Pin, 1);
	  HAL_Delay(300);
	  uint8_t res = MEAS_meas_cm(); //MEAS_calculateAsPercent(MEAS_meas_cm());
	  	  	  ///HAL_UART_Transmit(&huart2, &(ram2Test), 1, HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(TRANSISTOR_GPIO_Port, TRANSISTOR_Pin, 0);
	  MEAS_TRIG_GPIO_Port->MODER &= ~(GPIO_MODER_MEAS_TRIG_PIN);
	  //HAL_UART_Transmit_IT(&huart1, "AT+TEST=TXLRPKT, \"00 AA 11 BB 22 CC\"\r\n", strlen("AT+TEST=TXLRPKT, \"00 AA 11 BB 22 CC\"\r\n"));
	  HAL_Delay(50);
	  char LoRaFrame[200];
	  uint8_t len = sprintf(LoRaFrame, "AT+TEST=TXLRSTR,\"val: %i\"\r\n", (int)res);
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)LoRaFrame, len);
	  HAL_Delay(1800);//HAL_Delay(1000);
	  	  	  	  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)LORA_TEST_STOP, strlen(LORA_TEST_STOP));
	  //turnOffUART_flag = 1;
	  if(1)
	  {
		  HAL_UART_Transmit_IT(&huart1, "AT+TEST=RXLRPKT\r\n", 17);
		  HAL_Delay(6000);
		  char* configDataRaw = strstr(LoRaBufLongRX, "56543D");
		  if(configDataRaw)
		  {
			char* configData = configDataRaw + 6;
			char hexFromFrame [9];
			for(uint8_t i = 0; i<8; i++)
			{
			  if((configData[i] >= '0' && configData[i] <= '9') || (configData[i] >= 'A' && configData[i] <= 'F'))
				hexFromFrame[i] = configData[i];
			  else
				break;
			}
			char hexToAscii[5] = {0,0,0,0,0};
			for(uint8_t i = 0; i<4; i++)
			{
				if(hexFromFrame[2*i] <= '9' && hexFromFrame[2*i] >= '0')
				{
				  hexToAscii[i] += 16 * (hexFromFrame[2*i] - '0');
				}
				if(hexFromFrame[2*i + 1] <= '9' && hexFromFrame[2*i + 1] >= '0')
				{
				  hexToAscii[i] += (hexFromFrame[2*i+1] - '0');
				}
			}
			uint16_t valFromUser = atoi(hexToAscii);
			measurementsInterval = 86400/valFromUser;
			HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
			if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, measurementsInterval, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
			{
			  Error_Handler();
			}
		  }
		  HAL_UART_Transmit_IT(&huart1,"AT+TEST=TXLRSTR,\"ok\"\r\n",strlen("AT+TEST=TXLRSTR,\"ok\"\r\n"));
		  HAL_Delay(500);

	  }
// 	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)LORA_LOWPOWER_EXTR_ON, strlen(LORA_LOWPOWER_EXTR_ON));
	  HAL_Delay(200);
	  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)LORA_LOWPOWER_EXTR_ON, strlen(LORA_LOWPOWER_EXTR_ON));
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	  HAL_Delay(200);
	  HAL_PWR_EnterSTANDBYMode();
	  //HAL_UART_Transmit_IT(&huart1, LORA_RSSI, strlen(LORA_RSSI));
//	  HAL_Delay(1000);
	  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	 // HAL_UART_Transmit_IT(&huart1, "AT+TEST=RXLRPKT\r\n", 17);

//	  if(!(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)))
//	  {
//		  buttonPressed++;
//	  }
//	  else
//	  {
//		  if(buttonPressed > 9)
//		  {
//			  MEAS_measMaxDistance();
//		  }
//		  else if(buttonPressed>4)
//		  {
//			  MEAS_measMinDistance();
//		  }
//
//		  buttonPressed = 0;
//	  }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		if(turnOffUART_flag)
		{
			turnOffUART_flag = 0;
			HAL_UART_DeInit(&huart1);
			GPIO_BufStruct.Pin =GPIO_PIN_9|GPIO_PIN_10;
			HAL_GPIO_Init(GPIOA, &GPIO_BufStruct);
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &huart1)
	{
		LoRaBufLongRX[LoRaBufPosRX++] = LoRaBufRX;
		HAL_UART_Receive_IT(&huart1, &LoRaBufRX, 1);

		HAL_UART_Transmit(&huart2, &LoRaBufRX, 1, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart2, "\r\n", 2, HAL_MAX_DELAY);
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
