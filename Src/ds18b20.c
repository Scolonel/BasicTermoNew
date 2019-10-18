#include "ds18b20.h"
//--------------------------------------------------
uint8_t LastDeviceFlag;
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t ROM_NO[8];
extern uint8_t Dev_ID[8][8];
extern uint8_t Dev_Cnt;
//--------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
micros *= (SystemCoreClock / 1000000) / 9;
/* Wait till done */
while (micros--) ;
}
//--------------------------------------------------
void port_init(void)
{
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
  GPIOB->CRH |= GPIO_CRL_MODE6;
  GPIOB->CRH |= GPIO_CRL_CNF6_0;
  GPIOB->CRH &= ~GPIO_CRL_CNF6_1;
}
//--------------------------------------------------
uint8_t ds18b20_Reset(void)
{
  uint16_t status;
//     // set Baudrate 9600, send 0xF0, 
//    huart1.Instance->BRR = 0x4E8;
//    TxBufReq[0] = 0xF0;
//    HAL_UART_Receive_DMA(&huart1, RxBufAns, 1);
//  // пока бросаем в UART
//    HAL_UART_Transmit_DMA(&huart1, TxBufReq, 1);
//    HAL_Delay(50);
//    if(RxBufAns[0] == 0xE0)
//    status = 0;
//    else
//    status = 1;
//    RxBufAns[0]= 0;

	GPIOB->ODR &= ~GPIO_ODR_ODR6;//низкий уровень
  DelayMicro(485);//задержка как минимум на 480 микросекунд
  GPIOB->ODR |= GPIO_ODR_ODR6;//высокий уровень
  DelayMicro(65);//задержка как минимум на 60 микросекунд
  status = GPIOB->IDR & GPIO_IDR_IDR6;//проверяем уровень
  HAL_Delay(2);
  //DelayMicro(500);//задержка как минимум на 480 микросекунд
  //(на всякий случай подождём побольше, так как могут быть неточности в задержке)
  return (status ? 1 : 0);//вернём результат
}
//----------------------------------------------------------
uint8_t ds18b20_ReadBit(void)
{
  uint8_t bit = 0;
//  TxBufReq[0]=0xff;
//      // set Baudrate 115200, send 0xFF, 
//    huart1.Instance->BRR = 0x68;
//    HAL_UART_Receive_DMA(&huart1, RxBufAns, 1);
//  // пока бросаем в UART
//    HAL_UART_Transmit_DMA(&huart1, TxBufReq, 1);
// //HAL_Delay(1);
//    if(RxBufAns[0]==0xFF)
//      bit = 0;
//    else
//      bit = 1;
  GPIOB->ODR &= ~GPIO_ODR_ODR6;//низкий уровень
  DelayMicro(2);
	GPIOB->ODR |= GPIO_ODR_ODR6;//высокий уровень
	DelayMicro(13);
	bit = (GPIOB->IDR & GPIO_IDR_IDR6 ? 1 : 0);//проверяем уровень	
	DelayMicro(45);
  return bit;
}
//-----------------------------------------------
uint8_t ds18b20_ReadByte(void)
{
  uint8_t data = 0;
  for (uint8_t i = 0; i <= 7; i++)
  data += ds18b20_ReadBit() << i;
  return data;
}
//-----------------------------------------------
void ds18b20_WriteBit(uint8_t bit)
{
//    if (bit) 
//     TxBufReq[0]=0xFF;
//    else
//     TxBufReq[0]=0x00;
//      // set Baudrate 115200, send bit, 
//    huart1.Instance->BRR = 0x68;
//    HAL_UART_Receive_DMA(&huart1, RxBufAns, 1);
//  // пока бросаем в UART
//
//    HAL_UART_Transmit_DMA(&huart1, TxBufReq, 1);
//    // HAL_Delay(1); 
  GPIOB->ODR &= ~GPIO_ODR_ODR6;
  DelayMicro(bit ? 3 : 65);
  GPIOB->ODR |= GPIO_ODR_ODR6;
  DelayMicro(bit ? 65 : 3);
}
//-----------------------------------------------
void ds18b20_WriteByte(uint8_t dt)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    ds18b20_WriteBit(dt >> i & 1);
    //Delay Protection
    DelayMicro(15);
  }
}
//-----------------------------------------------
uint8_t ds18b20_SearhRom(uint8_t *Addr)
{
  uint8_t id_bit_number;
  uint8_t last_zero, rom_byte_number, search_result;
  uint8_t id_bit, cmp_id_bit;
  uint8_t rom_byte_mask, search_direction;
  //проинициализируем переменные
  id_bit_number = 1;
  last_zero = 0;
  rom_byte_number = 0;
  rom_byte_mask = 1;
  search_result = 0;
	if (!LastDeviceFlag)
	{
		ds18b20_Reset();
		ds18b20_WriteByte(0xF0);
	}
	do
  {
		id_bit = ds18b20_ReadBit();
		cmp_id_bit = ds18b20_ReadBit();
		if ((id_bit == 1) && (cmp_id_bit == 1))
			break;
		else
		{
			if (id_bit != cmp_id_bit)
				search_direction = id_bit; // bit write value for search
			else
			{
				if (id_bit_number < LastDiscrepancy)
					search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
				else
					search_direction = (id_bit_number == LastDiscrepancy);
				if (search_direction == 0)
				{
					last_zero = id_bit_number;
					if (last_zero < 9)
					LastFamilyDiscrepancy = last_zero;
				}
			}
			if (search_direction == 1)
				ROM_NO[rom_byte_number] |= rom_byte_mask;
			else
				ROM_NO[rom_byte_number] &= ~rom_byte_mask;
			ds18b20_WriteBit(search_direction);
			id_bit_number++;
			rom_byte_mask <<= 1;
			if (rom_byte_mask == 0)
			{
				rom_byte_number++;
				rom_byte_mask = 1;
			}
		}
  } while(rom_byte_number < 8); // считываем байты с 0 до 7 в цикле
	if (!(id_bit_number < 65))
  {
	  // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
		LastDiscrepancy = last_zero;
		// check for last device
		if (LastDiscrepancy == 0)
			LastDeviceFlag = 1;
		search_result = 1;	
  }
	if (!search_result || !ROM_NO[0])
	{
		LastDiscrepancy = 0;
		LastDeviceFlag = 0;
		LastFamilyDiscrepancy = 0;
		search_result = 0;
	}
	else
  {
    for (int i = 0; i < 8; i++) Addr[i] = ROM_NO[i];
  }	
  return search_result;
}
//-----------------------------------------------
uint8_t ds18b20_init(uint8_t mode)
{
	int i = 0, j=0;
  uint8_t dt[8];
  if(mode==SKIP_ROM)
  {
		if(ds18b20_Reset()) return 1;
		//SKIP ROM
		ds18b20_WriteByte(0xCC);
		//WRITE SCRATCHPAD
		ds18b20_WriteByte(0x4E);
		//TH REGISTER 100 градусов
		ds18b20_WriteByte(0x64);
		//TL REGISTER - 30 градусов
		ds18b20_WriteByte(0x9E);
		//Resolution 12 bit
		ds18b20_WriteByte(RESOLUTION_12BIT);
  }
	else
	{
		//if(ds18b20_Reset()) return 1;
		for(i=1;i<=8;i++)
		{
          HAL_Delay(200);
			if(ds18b20_SearhRom(dt))
			{
				Dev_Cnt++;
				memcpy(Dev_ID[Dev_Cnt-1],dt,sizeof(dt));
			}
			else break;
		}
		for(i=1;i<=Dev_Cnt;i++)
		{
			if(ds18b20_Reset()) return 1;
			//Match Rom
			ds18b20_WriteByte(0x55);
			for(j=0;j<=7;j++)
			{
				ds18b20_WriteByte(Dev_ID[i-1][j]);
			}
			//WRITE SCRATCHPAD
			ds18b20_WriteByte(0x4E);
			//TH REGISTER 100 градусов
			ds18b20_WriteByte(0x64);
			//TL REGISTER - 30 градусов
			ds18b20_WriteByte(0x9E);
			//Resolution 12 bit
			ds18b20_WriteByte(RESOLUTION_9BIT);
		}
	}
  return 0;
}
//----------------------------------------------------------
void ds18b20_MeasureTemperCmd(uint8_t mode, uint8_t DevNum)
{
	int i = 0;
  ds18b20_Reset();
  if(mode==SKIP_ROM)
  {
    //SKIP ROM
    ds18b20_WriteByte(0xCC);
  }
	else
	{
		//Match Rom
		ds18b20_WriteByte(0x55);
		for(i=0;i<=7;i++)
		{
			ds18b20_WriteByte(Dev_ID[DevNum][i]);
		}
	}
  //CONVERT T
  ds18b20_WriteByte(0x44);
}
//----------------------------------------------------------
void ds18b20_ReadStratcpad(uint8_t mode, uint8_t *Data, uint8_t DevNum)
{
  uint8_t i;
  ds18b20_Reset();
  if(mode==SKIP_ROM)
  {
    //SKIP ROM
    ds18b20_WriteByte(0xCC);
  }
	else
	{
		//Match Rom
		ds18b20_WriteByte(0x55);
		for(i=0;i<=7;i++)
		{
			ds18b20_WriteByte(Dev_ID[DevNum][i]);
		}
	}
  //READ SCRATCHPAD
  ds18b20_WriteByte(0xBE);
  for(i=0;i<8;i++)
  {
    Data[i] = ds18b20_ReadByte();
  }
}
//----------------------------------------------------------
uint8_t ds18b20_GetSign(uint16_t dt)
{
  //Проверим 11-й бит
  if (dt&(1<<11)) return 1;
  else return 0;
}
//----------------------------------------------------------
float ds18b20_Convert(uint16_t dt)
{
  float t;
  
  //t = (float) ((dt&0x07FF)>>4); //отборосим знаковые и дробные биты
  //Прибавим дробную часть
  //t += (float)(dt&0x000F) / 16.0f;
   
    t = (dt/8)*0.5  ;
  return t;
}
//----------------------------------------------------------
