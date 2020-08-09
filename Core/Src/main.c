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
#include "string.h"
#include "platform/deca_device_api.h"
#include "platform/deca_regs.h"
#include "platform/stdio.h"
#include "platform/deca_spi.h"
#include "platform/port.h"
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

void long_flash()
{
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_Delay(1000);
}
void short_flash()
{
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_Delay(200);
}
void debugPrint(char _out[])
{
	 HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 200);
	 //char newline[2] = "\r\n";
	 //HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 200);
}
void debugInt32Hexln(uint32_t uint32data)
{
	char buff[8];
	sprintf(buff,"%08X",uint32data);
	HAL_UART_Transmit(&huart1, buff, 8, 200);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 200);
}
void debugInt16Hexln(uint16_t data)
{
	char buff[4];
	sprintf(buff,"%04X",data);
	HAL_UART_Transmit(&huart1, buff, 4, 200);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 200);
}
void debugInt8Hexln(uint8_t uint8data)
{
	char buff[2];
	sprintf(buff,"%02X",uint8data);
	HAL_UART_Transmit(&huart1, buff, 2, 200);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 200);
}
void debugInt8Hex(uint8_t uint8data)
{
	char buff[2];
	sprintf(buff,"%02X",uint8data);
	HAL_UART_Transmit(&huart1, buff, 2, 200);
}
uint8 DW1000_init_fail()
{
	uint8 init_fail = 1;
	uint8 const fail_count = 20;
	for(uint8 i=0;i<fail_count;i++){
	  if (!(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR))
	  {
		  debugPrint("INIT SUCCESS\r\n");
		  init_fail = 0;
		  break;
	  }
	}
	return init_fail;
}
void DW1000_initiate()
{
	while(dwt_read16bitoffsetreg(AGC_CTRL_ID,AGC_TUNE1_OFFSET)!=0x8870)
		dwt_write16bitoffsetreg(AGC_CTRL_ID,AGC_TUNE1_OFFSET,0x8870);
	while(dwt_read32bitoffsetreg(DRX_CONF_ID,DRX_TUNE2_OFFSET)!=0x311A002D)
		dwt_write32bitoffsetreg(DRX_CONF_ID,DRX_TUNE2_OFFSET,0x311A002D);
	while(dwt_read8bitoffsetreg(LDE_IF_ID,LDE_CFG1_OFFSET)!=0x6D)
		dwt_write8bitoffsetreg(LDE_IF_ID,LDE_CFG1_OFFSET,0x6D);
	while(dwt_read16bitoffsetreg(LDE_IF_ID,LDE_CFG2_OFFSET)!=0x1607)
		dwt_write16bitoffsetreg(LDE_IF_ID,LDE_CFG2_OFFSET,0x1607);
	while(dwt_read32bitreg(TX_POWER_ID)!=0x0E082848)
		dwt_write32bitreg(TX_POWER_ID,0x0E082848);
	while(dwt_read32bitreg(SYS_CFG_ID)!=0x00001600)
		dwt_write32bitreg(SYS_CFG_ID,0x00001600);
	while(dwt_read8bitoffsetreg(FS_CTRL_ID,FS_PLLTUNE_OFFSET)!=0xBE)
		dwt_write8bitoffsetreg(FS_CTRL_ID,FS_PLLTUNE_OFFSET,0xBE);
}
void spi_test()
{
 uint8 dataA[10] = {0x11,0x22,0x33,0x44,0x55,0x66,0x78,0x9A,0xBC,0xDE};
 uint8 dataB[10];
 dwt_writetodevice(0x21, 0, 10, &dataA);
 while(1)
 {
	  HAL_Delay(1000);
	  for(uint8 i =0;i<10;i++)dataB[i] = 0x00;
	  dwt_readfromdevice(0x21, 0 , 10,&dataB);
	  debugInt8Hex(dataB[0]);
	  debugInt8Hex(dataB[1]);
	  debugInt8Hex(dataB[2]);
	  debugInt8Hex(dataB[3]);
	  debugInt8Hex(dataB[4]);
	  debugInt8Hex(dataB[5]);
	  debugInt8Hex(dataB[6]);
	  debugInt8Hex(dataB[7]);
	  debugInt8Hex(dataB[8]);
	  debugInt8Hex(dataB[9]);
	  debugPrint("\r\n");
 }
}
/*-------------------------------------define ---------*/

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
/*
D 0x44
E 0x45
C 0x43
A 0x41
W 0x57
A 0x41
V 0x56
E 0x45
*/
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 2000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint32 u32;
  port_set_dw1000_slowrate();
  //port_set_dw1000_fastrate();
  debugPrint("\r\n===============Restart_TX V1.0.0=================\r\n");
  while(1)
  {
	  //DWReset();
	  debugInt32Hexln(dwt_read32bitreg(SYS_STATUS_ID));
	  reset_DW1000();
	  debugInt32Hexln(dwt_read32bitreg(DEV_ID_ID));
	  debugInt32Hexln(dwt_read32bitreg(SYS_STATUS_ID));
	  if(DW1000_init_fail())continue; //1 means initiate fails
	  HAL_Delay(1000);
	  debugInt32Hexln(dwt_read32bitreg(SYS_STATUS_ID));
//	  debugInt32Hexln(u32);
//	  debugInt32Hexln(u32);

	  //port_set_dw1000_fastrate();
	  //debugInt32Hexln(dwt_read32bitoffsetreg(EXT_SYNC_ID,EC_CTRL_OFFSET));
	  //reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */

	  DW1000_initiate();
	  debugInt16Hexln(dwt_read16bitoffsetreg(AGC_CTRL_ID,AGC_TUNE1_OFFSET));
	  debugInt32Hexln(dwt_read32bitoffsetreg(DRX_CONF_ID,DRX_TUNE2_OFFSET));
	  debugInt8Hexln(dwt_read8bitoffsetreg(LDE_IF_ID,LDE_CFG1_OFFSET));
	  debugInt16Hexln(dwt_read16bitoffsetreg(LDE_IF_ID,LDE_CFG2_OFFSET));
	  debugInt32Hexln(dwt_read32bitreg(TX_POWER_ID));
	  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID)&0x000F0000);
	  debugInt32Hexln(dwt_read32bitreg(SYS_STATUS_ID));
	  debugInt8Hexln(dwt_read8bitoffsetreg(FS_CTRL_ID,FS_PLLTUNE_OFFSET));

	  debugInt32Hexln(dwt_read32bitoffsetreg(EXT_SYNC_ID,EC_CTRL_OFFSET));
	  debugPrint("******************\r\n");
	  //spi_test();
	  while(1);



	  /* Configure DW1000. See NOTE 7 below. */
	  dwt_configure(&config);

	  HAL_Delay(200);
	  u32 = 0;
	  u32 = dwt_read32bitreg(SYS_STATUS_ID);
	  //debugInt32Hexln(u32);
	  if (u32 != 0x02800002)continue;

	  uint32 totcheck = 0;

	  /* Loop forever sending frames periodically. */
	 while(1)
	 {
		 /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
		 //reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
		 //if(DW1000_init() == 1)continue;
		 //dwt_configure(&config);
		 //HAL_Delay(200);
		 //u32 = 0;
		 //u32 = dwt_read32bitreg(SYS_STATUS_ID);
		 //if (u32 != 0x02800002)continue;

		 dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
		 dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

		 totcheck = 0;
		 while ( u32 != 0x02800000)
		 {
		 	 dwt_write32bitreg(SYS_STATUS_ID, 0x000000F0);
			 u32 = dwt_read32bitreg(SYS_STATUS_ID);
			 totcheck ++;
			 if (totcheck >10)break;
		 }
//		 if (u32 != 0x02800002)continue;

		 /* Start transmission. */
		 debugPrint("*");
		 debugInt32Hexln(dwt_read32bitreg(SYS_STATUS_ID));
		 dwt_starttx(DWT_START_TX_IMMEDIATE);
//		  while(1)
//		  {
//			  dwt_starttx(DWT_START_TX_IMMEDIATE);
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugInt32Hexln(dwt_read32bitreg(SYS_STATE_ID));
//			  debugPrint("%%%%%%%%%%%\r\n");
//			  HAL_Delay(1000);
//		  }

		 /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
		  * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
		  * function to access it.*/
		 u32 = 0;
		 u32 = dwt_read32bitreg(SYS_STATUS_ID);
		 while ( (u32 & SYS_STATUS_TXFRS) != SYS_STATUS_TXFRS)
		 {
			 //debugPrint("not ok\r\n");
			 u32 = dwt_read32bitreg(SYS_STATUS_ID);
			 totcheck ++;
			 if (totcheck >1024)break;
		 }
		 if ((u32 & SYS_STATUS_TXFRS) != SYS_STATUS_TXFRS)continue;

		 debugPrint("^");
		 debugInt32Hexln(u32);

		 debugPrint("--ok!\r\n");

		 /* Clear TX frame sent event. */
		 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

		 /* Execute a delay between transmissions. */
		 Sleep(TX_DELAY_MS);
		 short_flash();

		 /* Increment the blink frame sequence number (modulo 256). */
		 tx_msg[BLINK_FRAME_SN_IDX]++;
	 }

	  while(1);
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  huart1.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DW_RESET_Pin|DW_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_IRQn_Pin */
  GPIO_InitStruct.Pin = DW_IRQn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_RESET_Pin */
  GPIO_InitStruct.Pin = DW_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_NSS_Pin */
  GPIO_InitStruct.Pin = DW_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_NSS_GPIO_Port, &GPIO_InitStruct);

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
