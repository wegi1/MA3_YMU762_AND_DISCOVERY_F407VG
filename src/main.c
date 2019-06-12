/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"



#include "mamachdep.h"
#include "masound.h"

#include "MMF.h"

/* Private variables ---------------------------------------------------------*/
SRAM_HandleTypeDef hsram1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);

#if 1
extern UINT32 ulPos;
extern YMU262_LOG_ENTRY logbuf[LOG_ENTRIES_MAX];
#endif

extern void MaDevDrv_IntHandler(void);

#define YMU762_RST(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_##x)

#define PAUSE HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)

#define LED(x) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_##x)


void delay_ms(volatile uint32_t d)
{
    HAL_Delay(d);
}

void YMU762_Reset(void)
{
    YMU762_RST(RESET);
    delay_ms(10);
    YMU762_RST(SET);
    delay_ms(1);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint8_t t=0;

    /* Prevent unused argument(s) compilation warning */
    UNUSED(GPIO_Pin);

    MaDevDrv_IntHandler();

    t^=1;

    if(t) {
        LED(SET);
    } else {
       LED(RESET);
    }

}


signed long CallBack(unsigned char id)
{
 /*
 switch(id)
 {
  case MASMW_REPEAT:
  break;
  case MASMW_END_OF_SEQUENCE:
  break;
  default:
  break;
 }
 */
 return MASMW_SUCCESS;
}

extern UINT32 ulCount;
extern UINT32 ulCountRdSt;
extern UINT32 ulCountRdDa;
extern UINT32 ulCountWrSt;
extern UINT32 ulCountWrDa;

extern UINT32 ulCountNonRep;
extern UINT32 ulPos;


#define USB_OUT_PACKET_MAX 512
#define SINGLE_LOG_ENTRY 13

// -10 because of the "timestamp:" preamble
#define NR_OF_LOG_ENTRIES_IN_PACKET_MAX ((USB_OUT_PACKET_MAX - SINGLE_LOG_ENTRY - 1)/SINGLE_LOG_ENTRY)

char strTmp[USB_OUT_PACKET_MAX];
int nLen = 0;
int delta = 0;
uint32_t lastPos = 0;
uint32_t firstPos = 0;
uint32_t tickNow = 0;
uint32_t tickFirst = 0;
int thisChunk = 0;
uint32_t CDC_requests_sent = 0;
int32_t func = 0;
int32_t file = 0;
uint8_t volume = 0;


int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_FSMC_Init();

    YMU762_Reset();

    while(! PAUSE);

    MaSound_Initialize();

    MaSound_DeviceControl(MASMW_HP_VOLUME, 0, 31, 31);
    MaSound_DeviceControl(MASMW_EQ_VOLUME, 0, 0, 0);
    MaSound_DeviceControl(MASMW_SP_VOLUME, 0, 0, 0);

    func=MaSound_Create(MASMW_CNVID_MMF /*MASMW_CNVID_MID*/);
    file=MaSound_Load(func, (uint8_t*)MMF, sizeof(MMF), 1, CallBack, NULL);

    MaSound_Open(func,file,0,NULL);

    volume=127; //Max 0 dB
    MaSound_Control(func,file,MASMW_SET_VOLUME,&volume,NULL);

    MaSound_Standby(func,file,NULL);

    uint32_t tickFirst = HAL_GetTick();

    MaSound_Start(func,file,0,NULL); //Play Loop

    while(1)
    {
        firstPos = ulPos;                                                   // current log position (from mamachdep.c)

        if(firstPos >= lastPos)                                             // some new samples stored (no boundary crossed)
            delta = firstPos - lastPos;
        else
            delta = LOG_ENTRIES_MAX - lastPos + firstPos;                   // boundary crossed


        // if TX busy or no data to be sent - check again in 1 ms
        if( (CDC_CheckTransmitPossible() == USBD_BUSY) || (delta == 0) )
        {
            //delay_ms(1);
            continue;
        }

        tickNow = HAL_GetTick();
        nLen = sprintf(strTmp, ":%08X\r", tickNow - tickFirst);

        thisChunk = MIN(delta, NR_OF_LOG_ENTRIES_IN_PACKET_MAX);

        while(delta > 0)
        {
            for(int i=0; i< thisChunk; i++)
            {
                nLen += sprintf(strTmp + nLen,"%04X%04X%02X%02X\r", firstPos, logbuf[firstPos].ushCnt, logbuf[firstPos].flag, logbuf[firstPos].data);

                firstPos++;
                if(firstPos >= LOG_ENTRIES_MAX)
                {
                    firstPos = 0;
                }
            }

            CDC_Transmit_FS(strTmp,nLen);

            nLen = 0;

            while( CDC_CheckTransmitPossible() == USBD_BUSY )
            {
                //delay_ms(1);
            }

            CDC_requests_sent++;

            delta -= thisChunk;
        }


        lastPos = firstPos;

/*
        if(PAUSE)
        {
            MaSound_Pause(func,file,NULL);

            {
                uint32_t tickNow = HAL_GetTick();
                char strTmp[128];
                int nLen;
                //nLen = sprintf(strTmp,"T:%d, C-TOT:%d RD-ST:%d RD-DA:%d WR-ST:%d WR-DA:%d\r", tickNow-tickFirst, ulCount, ulCountRdSt, ulCountRdDa, ulCountWrSt, ulCountWrDa);
                nLen = sprintf(strTmp,"T:%d, C-TOT:%d C-NONR:%d\r", tickNow-tickFirst, ulCount, ulCountNonRep);
                CDC_Transmit_FS(strTmp,nLen);
            }

            delay_ms(100);
            while(PAUSE);
            delay_ms(100);
            while(!PAUSE);
            delay_ms(100);
            while(PAUSE);
            delay_ms(100);

            MaSound_Restart(func,file,NULL);
        }
*/
    }
}



//---------------------------------------------------------------------------------------------------------------------------------------------------



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
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
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nRST_GPIO_Port, nRST_Pin, GPIO_PIN_SET);

  /*Configure LED pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PUSHBUTT_Pin */
  GPIO_InitStruct.Pin = PUSHBUTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PUSHBUTT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nIRQ_Pin */
  GPIO_InitStruct.Pin = nIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nIRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nRST_Pin */
  GPIO_InitStruct.Pin = nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 0;
  Timing.AddressHoldTime = 0;
  Timing.DataSetupTime = 6;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 0;
  Timing.DataLatency = 0;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
