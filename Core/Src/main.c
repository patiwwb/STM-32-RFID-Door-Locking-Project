/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
//#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "rc522.h"
#include "retarget.h"
#include "mfrc522.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[20] = {0};
flag_test = 0;

uint8_t         i; //for
uint8_t         differentRFID; //czy nowe RFID jest inne? (multiple readings protect)
uint8_t         txBufInd;
uint8_t         comand;
uint8_t         txBuffer[18] = "ID: 00000000  "; //id w postaci char *
uint8_t         retstr[10]; //powrót z funkcji konwersji hex_dec
uint8_t         rxBuffer[8]; //ID w postacji stringu do odczytania (heksadecynalny)
uint8_t         lastID[4]; //poprzednio wczytany ID w postaci binarnej
uint8_t         str[MFRC522_MAX_LEN]; //aktualnie wczytany id w postaci binarnej (4 bajty, ale miejsca na 16)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
#include <stdio.h>
#include <string.h>
volatile char t[100];
char buf_printf[100]; //max printf length
#define printfx(f_, ...) snprintf(buf_printf, 100, (f_), ##__VA_ARGS__); \
HAL_UART_Transmit(&huart3, (uint8_t*)buf_printf, strlen(buf_printf), 1000);


volatile uint8_t rc522interrupt = 0; // wystąpiło przerwanie RC522 trzeba obsłużyć w loop()



//FUNKCJE CZYTNIKA
uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
uint8_t MFRC522_ReadRegister(uint8_t addr);
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
void MFRC522_CalulateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
uint8_t MFRC522_SelectTag(uint8_t* serNum);
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
void MFRC522_Init(void);
void MFRC522_Reset(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);
void MFRC522_Halt(void);

//////////////

void char_to_hex(uint8_t data);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
 // RC522_Init();
  HAL_UART_Transmit(&huart2, (uint8_t *)" UART pOLLING METHOD\r\n", sizeof(" UART pOLLING METHOD\r\n"), 300);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, 1);
	  //	  HAL_Delay(500);
	  //	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, 0);
	  //	  HAL_Delay(500);
//	  	  if(!flag_test)
//	  	  {
//	  		  HAL_UART_Transmit(&huart2,(uint8_t *) "UART2 Interrupt\r\n", sizeof("UART2 Interrupt\r\n"), 300);
//	  		  flag_test = 1;
//	  	  }
//	  	  uint8_t RC522version = MFRC522_ReadRegister(MFRC522_REG_VERSION);
//	  	  printf("RC522 Version (92 = version 2, 91 = version 1, ff = rst error): %#08x\r\n", RC522version );
//	      HAL_Delay(1000);
//	      printf("\r\nYes ");

		  //ReaderCard();


	  //	  HAL_UART_Receive(&huart2, rx_buffer,20, 300);
	  //	  HAL_UART_Transmit(&huart2, rx_buffer, 20, 300);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t RC522version = MFRC522_ReadRegister(MFRC522_REG_VERSION);
	  printf("RC522 Version (92 = version 2, 91 = version 1, ff = rst error): %#08x\r\n", RC522version );
	  if (!MFRC522_Request(PICC_REQIDL, str))//str - nasz klucz //PICC_REQIDL find the antenna area does not enter hibernation
	                          {
	                                  if (!MFRC522_Anticoll(str))//str nasz klucz[16] ANTI COLLISION
	                                  {
	                                          //TU ODCZYTAł nowy klucz ID
	                                          differentRFID = 0; //bool czy kolejny wczytany klucz inny jest od poprzedniego? Tylko wtedy działaj (multiple readings)
	                                          txBufInd = 9; //indeks w stringu od którego piszemy ID: ...

	                                          for (i=0; i<4 && !differentRFID; i++)
	                                                  if (lastID[i] != str[i])
	                                                          differentRFID = 1; //sprawdzenie czy wczytany teraz klucz != poprzednio wczytanego

	                                          if (differentRFID)
	                                          { //to się wywołuje jeśli kod RFID jest inny nniż do tej pory
	                                                  for (i=0; i<4; i++)
	                                                          lastID[i] = str[i]; //zapamiętaj ostatni nowy ID w str
	                                                  for (i=0; i<4; i++)
	                                                  {
	                                                          char_to_hex(str[i]); //tu zapisuje w systemie 16 klucz do stringu z formatu bitowego na liczby 16 = 4 bajty
	                                                          txBuffer[txBufInd] = retstr[0]; //retstr to return string z funkcji char_to_hex
	                                                          txBufInd++;
	                                                          txBuffer[txBufInd] = retstr[1]; //retstr to return string z funkcji char_to_hex
	                                                          txBufInd++;
	                                                  }
	                                                  printf("RFID: %s \n\r", (char *)txBuffer);
	                                                  //HAL_UART_Transmit(&huart3, txBuffer, 18, 100);
	                                          }

	                                  }
	                          }

	                          HAL_Delay(10);

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  USART2->CR1 |= (USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RC522_RST_Pin */
  GPIO_InitStruct.Pin = RC522_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RC522_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC522_SDA_Pin */
  GPIO_InitStruct.Pin = RC522_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RC522_SDA_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, RC522_RST_Pin, GPIO_PIN_SET); //domyślnie RESET będzie i dlatego nie działa.

}

/* USER CODE BEGIN 4 */
// char number to string hex (FF) (Only big letters!)
void char_to_hex(uint8_t data) {
        uint8_t digits[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

        if (data < 16) {
                retstr[0] = '0';
                retstr[1] = digits[data];
        } else {
                retstr[0] = digits[(data & 0xF0)>>4];
                retstr[1] = digits[(data & 0x0F)];
        }
}


void irqHandlerRC522(void)
{//niepodłączone
   rc522interrupt = 1;
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

