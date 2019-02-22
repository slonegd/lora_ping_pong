extern "C" {
   #include "stm32l1xx.h"
   #include "utilities.h"
   #include "gpio.h"
   #include "adc.h"
   #include "spi.h"
   #include "i2c.h"
   #include "uart.h"
   #include "timer.h"
   #include "board-config.h"
   #include "lpm-board.h"
   #include "rtc-board.h"

   #if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
      #include "sx126x-board.h"
   #elif defined( SX1272MB2DAS)
      #include "sx1272-board.h"
   #elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
      #include "sx1276-board.h"
   #endif
   #include "board.h"

   #include "delay.h"
   #include "timer.h"
   #include "radio.h"
}


namespace lmn {

struct Radio {
   const Radio_s& radio;

   Radio (SpiId_t spi, PinNames mosi, PinNames miso, PinNames clk)
   : radio {::Radio}
   {
      HAL_Init();


      // SystemClockConfig();
      RCC_OscInitTypeDef RCC_OscInitStruct;
      RCC_ClkInitTypeDef RCC_ClkInitStruct;
      RCC_PeriphCLKInitTypeDef PeriphClkInit;

      __HAL_RCC_PWR_CLK_ENABLE( );

      __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
      RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
      RCC_OscInitStruct.HSIState = RCC_HSI_ON;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
      RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
      RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
      RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
      RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
      RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;
      if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
      {
         assert_param( FAIL );
      }

      RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
      if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
      {
         assert_param( FAIL );
      }

      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
      PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
      if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
      {
         assert_param( FAIL );
      }

      HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

      HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

      // SysTick_IRQn interrupt configuration
      HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
      // SystemClockConfig();

      RtcInit();

      // BoardUnusedIoInit();
      HAL_DBGMCU_EnableDBGSleepMode( );
      HAL_DBGMCU_EnableDBGStopMode( );
      HAL_DBGMCU_EnableDBGStandbyMode( );
      // BoardUnusedIoInit();

      if (GetBoardPowerSource() == BATTERY_POWER){
         // Disables OFF mode - Enables lowest power mode (STOP)
         LpmSetOffMode (LPM_APPLI_ID, LPM_DISABLE);
      }
   #if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
      SpiInit (&SX126x.Spi, spi, mosi, miso, clk, NC);
      SX126xIoInit();
   #elif defined(SX1272MB2DAS)
      SpiInit (&SX1272.Spi, spi, mosi, miso, clk, NC);
      SX1272IoInit();
   #elif defined(SX1276MB1LAS) || defined(SX1276MB1MAS)
      SpiInit (&SX1276.Spi, spi, mosi, miso, clk, NC);
      SX1276IoInit();
   #endif
   }

};

} // namespace lmn {