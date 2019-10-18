/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// тест ѕ–ќ√–јћћј дл€ проверки возможностей по приему, передаче посылок в линии системы 1083
#include "ssd1306.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "ds18b20.h"

#define OW_0    0x00
#define OW_1    0xff
#define OW_R    0xff

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
/* Private variables ---------------------------------------------------------*/
const uint8_t convert_T[] = {
                OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
                OW_0, OW_0, OW_1, OW_0, OW_0, OW_0, OW_1, OW_0  // 0x44 CONVERT
//                OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, // 0xcc SKIP ROM
//                OW_0, OW_1, OW_0, OW_0, OW_0, OW_1, OW_0, OW_0  // 0x44 CONVERT
};
const uint8_t read_scratch[] = {
                OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
                OW_0, OW_1, OW_1, OW_1, OW_1, OW_1, OW_0, OW_1, // 0xbe READ SCRATCH
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R
};
const uint8_t read_ROM[] = {
                OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, // 0x33 get ROM
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R
};

uint8_t Str[30]; // строка дл€ вывода
uint8_t StartKeys; // состо€ние переключателей при старте
uint8_t RxBufCmd[256]; // буффер прин€той команды
uint8_t TxBufAns[256]; // буффер передачи в USB
uint8_t TxBufReq[256]; // буффер передачи в UART
uint8_t RxBufAns[256]; // буффер что прин€ли в ответ
uint16_t CountCmd; // длина прин€той команды и признак необходимости обработки
uint32_t Dummy;
uint32_t g_EnaTemp = 0;
// new cmd
uint8_t Dev_ID[8][8]={0};
uint8_t Dev_Cnt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// проверка что прин€ли по VCP 
void CheckRxVCP (uint16_t Size);
//получение состо€ни€ переключателей выбора номера абонента
uint8_t GetKeyStat (uint8_t Num);
// цикл измерени€ температуры
float GetTemp(void);
// поиск приборов по новому
void SrDev (void);
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
  SSD1306_I2C_ADDR = 0x78;
	uint8_t dt[8];
	volatile static uint16_t raw_temper;
	float temper;
    volatile uint8_t CntVrm=0;

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
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_2 ); // вроде как надо запустить таймеры
  // init oled
  uint8_t res = SSD1306_Init();
  sprintf((char*)Str,"HELLO! ver 5.0") ;
  //1.0 - принимает команды из линии и детектирует их, выводит построчно на экран
  SSD1306_GotoXY(0,53);
  SSD1306_Puts((void*)Str, &Font_7x10, 1);
  SSD1306_UpdateScreen();
 // получим состо€ние датчиков 
  //port_init();
  HAL_Delay(300);
  ds18b20_init(1);
  sprintf((char*)Str,"%d",Dev_Cnt) ;
  SSD1306_GotoXY(110,53);
  SSD1306_Puts((void*)Str, &Font_7x10, 1);
  	for(int i=0;i<Dev_Cnt;i++)
	{
		sprintf((char*)Str,"%02X%02X%02X%02X%02X%02X%02X%02X",
			Dev_ID[i][0], Dev_ID[i][1], Dev_ID[i][2], Dev_ID[i][3],
			Dev_ID[i][4], Dev_ID[i][5], Dev_ID[i][6], Dev_ID[i][7]);
  SSD1306_GotoXY(0,37+i*8);
  SSD1306_Puts((void*)Str, &Font_7x10, 1);
	}
 
  SSD1306_UpdateScreen();
  
  
  // пустое чтение дл€ инициализации приемеов!
  //Dummy = huart1.Instance->DR ;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(CountCmd) // есть команда надо обработать передать в линию
    {
      CheckRxVCP (CountCmd);
      CountCmd = 0;
    }
    for(int i=0;i<Dev_Cnt;i++)
    {
      ds18b20_MeasureTemperCmd(NO_SKIP_ROM, i);
    }
    HAL_Delay(800);
    for(int i=0;i<Dev_Cnt;i++)
    {
      ds18b20_ReadStratcpad(NO_SKIP_ROM, dt, i);
      raw_temper = ((uint16_t)dt[1]<<8)|dt[0];
      //if(ds18b20_GetSign(raw_temper)) c='-';
      //else c='+';
      temper = ds18b20_Convert(raw_temper);
      sprintf((char*)Str,"             ") ;
      SSD1306_GotoXY(0,18*i);
      SSD1306_Puts((void*)Str, &Font_11x18, 1);
      sprintf((char*)Str,"%c=%.1f",(i)?('I'):('0'), temper) ;
      //sprintf((char*)Str,"%c=%c%.1f",(i)?('I'):('0'),c, temper) ;
      // ¬ыводим что прин€ли
      SSD1306_GotoXY(0,18*i);
      SSD1306_Puts((void*)Str, &Font_11x18, 1);
    }
    SSD1306_UpdateScreen();
    HAL_Delay(150);
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
  // счетчик времени дл€ перезапуска прибора
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  if(CntVrm++ == 0) 
  {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    SrDev();
  }
      
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// цикл измерени€ температуры
float GetTemp(void)
{
  uint32_t SzTx, Data;
  int16_t Temp;
  HAL_StatusTypeDef StatRx, StatTx;
    // set Baudrate 9600, send 0xF0, RESET
//    huart1.Instance->BRR = 0x4E8;
//    TxBufReq[0] = 0xF0;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 1);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, TxBufReq, 1);
    HAL_Delay(40);
    if(RxBufAns[0] == 0xE0)
    {
      RxBufAns[0]= 0;
      //Dummy = huart1.Instance->DR ;
     // huart1.RxState = HAL_UART_STATE_READY;
    }
     else return -273.0;
  // запуск измерений, 115200
//    huart1.Instance->BRR = 0x68;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 16);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, (void*)convert_T, 16);// 16 byte 
    HAL_Delay(200);
    //huart1.RxState = HAL_UART_STATE_READY;
    HAL_Delay(1000);
    // set Baudrate 9600, send 0xF0, RESET
//    huart1.Instance->BRR = 0x4E8;
//    TxBufReq[0] = 0xF0;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 1);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, TxBufReq, 1);
    HAL_Delay(40);
    if(RxBufAns[0] == 0xE0)
    {
      RxBufAns[0]= 0;
     // huart1.RxState = HAL_UART_STATE_READY;
    }
     else return -273.0;
    //получаем температуру
    // set Baudrate 115200, send 0xF0, 
//    huart1.Instance->BRR = 0x68;
//    //TxBufReq[0] = 0xF0;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 32);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, (void*)read_scratch, 32);// read 32 bytes
    HAL_Delay(200);
    // попробуем разобрать что прин€ли
    //huart1.RxState = HAL_UART_STATE_READY;
    Data=0;
    for(int i=31; i>15;--i)
    {
      if(RxBufAns[i]==0xff) Data +=1;
      Data = Data<<1;
    }
    Data = Data>>1;
    Temp = (int16_t)(Data);
    return (Temp/8)*0.5  ;

}
// поиск приборов по новому
void SrDev (void)
{
  Dev_Cnt = 0;
  HAL_Delay(300);
  ds18b20_init(1);
  sprintf((char*)Str,"%d",Dev_Cnt) ;
  SSD1306_GotoXY(110,53);
  SSD1306_Puts((void*)Str, &Font_7x10, 1);
  for(int i=0;i<Dev_Cnt;i++)
  {
    sprintf((char*)Str,"%02X%02X%02X%02X%02X%02X%02X%02X",
			Dev_ID[i][0], Dev_ID[i][1], Dev_ID[i][2], Dev_ID[i][3],
			Dev_ID[i][4], Dev_ID[i][5], Dev_ID[i][6], Dev_ID[i][7]);
    SSD1306_GotoXY(0,37+i*8);
    SSD1306_Puts((void*)Str, &Font_7x10, 1);
  }
}

void CheckRxVCP (uint16_t Size)
{
  uint32_t SzTx, Data;
  int16_t Temp;
  volatile static uint64_t DataL;
  static uint32_t Dt[4];
  HAL_StatusTypeDef StatRx, StatTx;
  //uint32_t Dummy;
  // разбираем команду прин€тую по USB 
  if (!memcmp ((void*)&RxBufCmd[0], "RST",3))
  {
    // set Baudrate 9600, send 0xF0, 
//    huart1.Instance->BRR = 0x4E8;
//    TxBufReq[0] = 0xF0;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 1);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, TxBufReq, 1);
    HAL_Delay(40);
    if(RxBufAns[0] == 0xE0)
    SzTx = sprintf((char*)TxBufAns,"Ok") ;
    else
    SzTx = sprintf((char*)TxBufAns,"Err") ;
    RxBufAns[0]= 0;
    //Dummy = huart1.Instance->DR ;
    //huart1.RxState = HAL_UART_STATE_READY;
    //отсылаем назад, пока
  CDC_Transmit_FS (TxBufAns, SzTx);
      
  }
  
  if (!memcmp ((void*)&RxBufCmd[0], "STR",3))
  {
    // set Baudrate 115200, send 0xF0, 
//    huart1.Instance->BRR = 0x68;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 16);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, (void*)convert_T, 16);
    HAL_Delay(200);
    SzTx = sprintf((char*)TxBufAns,"Rx=%d Tx=%d", StatRx, StatTx) ;
    //отсылаем назад, пока
    CDC_Transmit_FS (RxBufAns, 16);
  }
  // запрос ROM
    if (!memcmp ((void*)&RxBufCmd[0], "ROM",3))
  {
    // set Baudrate 9600, send 0xF0, 
//    huart1.Instance->BRR = 0x68;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 72);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, (void*)read_ROM, 72); //0x33 Get ROM
    HAL_Delay(200);
    SzTx = sprintf((char*)TxBufAns,"Rx=%d Tx=%d", StatRx, StatTx) ;
        // попробуем разобрать что прин€ли
    DataL=0;
    for(int i=71; i>7;--i)
    {
      DataL = DataL<<1;
      if(RxBufAns[i]==0xff) DataL +=1;
    }
    Dt[0]=(uint32_t)((DataL>>8L)&0xFFFFFFFFL); // lbs
    Dt[1]=(uint32_t)((DataL>>40L)&0xFFFFL);    //mbs
    Dt[2]=(uint32_t)((DataL>>56L)&0xFFL); 
    Dt[3]=(uint32_t)((DataL)&0xFFL); 
  
    //SzTx = sprintf((char*)TxBufAns,"SN=%04X%08X CRC=%02X Cd=%02X", (uint64_t)((DataL&0x00FFFFFFFFFFFF00L)>>8L), (uint64_t)((DataL&0xFF00000000000000L)>>56L), (uint64_t)(DataL&0xFFL)) ;
    SzTx = sprintf((char*)TxBufAns,"SN=%04X%08X CRC=%02X Cd=%02X", Dt[1],Dt[0],Dt[2],Dt[3]) ;
    //отсылаем назад, пока
  CDC_Transmit_FS (TxBufAns, SzTx);
  }
// разрешение и запрещение измерений
    if (!memcmp ((void*)&RxBufCmd[0], "ENA",3))
  {
    g_EnaTemp = 1;
    SzTx = sprintf((char*)TxBufAns,"TempENA") ;
    //отсылаем назад, пока
    CDC_Transmit_FS (TxBufAns, SzTx );
  }
    // запрос ROM
    if (!memcmp ((void*)&RxBufCmd[0], "DIS",3))
  {
    g_EnaTemp = 0;
    SzTx = sprintf((char*)TxBufAns,"TempDIS") ;
    //отсылаем назад, пока
    CDC_Transmit_FS (TxBufAns, SzTx );
  }


  if (!memcmp ((void*)&RxBufCmd[0], "GET",3))
  {
    // set Baudrate 115200, send 0xF0, 
//    huart1.Instance->BRR = 0x68;
//    //TxBufReq[0] = 0xF0;
//    StatRx = HAL_UART_Receive_DMA(&huart1, RxBufAns, 32);
//  // пока бросаем в UART
//    StatTx = HAL_UART_Transmit_DMA(&huart1, (void*)read_scratch, 32);
    HAL_Delay(200);
    // попробуем разобрать что прин€ли
    Data=0;
    for(int i=31; i>15;--i)
    {
      if(RxBufAns[i]==0xff) Data +=1;
      Data = Data<<1;
    }
    Data = Data>>1;
    Temp = (int16_t)(Data);
    SzTx = sprintf((char*)TxBufAns,"Dt=%04X %.1f", Data, (Temp/8)*0.5 ) ;
    //отсылаем назад, пока
    CDC_Transmit_FS (RxBufAns, 32);
  }
  
   //huart1.RxState = HAL_UART_STATE_READY;

  // чистим строчку вывода
  sprintf((char*)Str,"                   ") ;
	SSD1306_GotoXY(0,53);
	SSD1306_Puts((void*)Str, &Font_7x10, 1);
  memcpy(Str, RxBufCmd, Size);
  Str[Size]=0;
  sprintf((char*)Str,"%s=%s",Str, TxBufAns) ;
  // ¬ыводим что прин€ли
  SSD1306_GotoXY(0,53);
  SSD1306_Puts((void*)Str, &Font_7x10, 1);
  SSD1306_UpdateScreen();
  
}
//получение состо€ни€ переключателей выбора номера абонента
uint8_t GetKeyStat (uint8_t Num)
{
  uint32_t Keys;
  switch(Num)
  {
  case 0: // номер абонента
    Keys = (((GPIOA->IDR>>1)&0x7F)^0x7F);
    break;
  case 1: // Code signal "free"
    Keys = (((GPIOB->IDR)&0x01)^0x01);
    break;
  default:// состо€ние ключей 1..8
    Keys = (((GPIOB->IDR)&0x01)^0x01)*128 + (((GPIOA->IDR>>1)&0x7F)^0x7F);
    break;
  }
  return   (uint8_t)(Keys);

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
