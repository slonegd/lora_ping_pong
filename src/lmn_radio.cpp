#include "lmn_radio.h"

extern "C" {
#include "stm32l1xx.h"
#include "timer.h"
#include "delay.h"
#include "lpm-board.h"
#include "rtc-board.h"

#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    #include "sx126x-board.h"
#elif defined( SX1272MB2DAS)
    #include "sx1272-board.h"
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    #include "sx1276-board.h"
#endif

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

void BoardLowPowerHandler( void )
{
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending 
     * and cortex will not enter low power anyway
     */

    LpmEnterLowPower( );

    __enable_irq( );
}

static void BoardUnusedIoInit();
static void SystemClockReConfig();
static void SystemClockConfig();

static void tx_done_callback();
static void rx_done_callback( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void tx_timeout_callback();
static void rx_timeout_callback();
static void rx_error_callback();
/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;
} // extern "C" {




namespace lmn {

Radio* Radio::pointer {nullptr};

Radio::Radio (
      SPI spi
    , MOSI mosi
    , MISO miso
    , CLK clk
    , Region_frequency frequency
    , Power power_
    , Bandwidth bandwidth
    , Spreading_factor spreading_factor
    , Coding_rate coding_rate
    , Preambula_length preambula_length_
    , Fix_length_payload fix_length_payload_
    , IQ_inversion iq_inversion_
    , Symbol_timeout symbol_timeout_
    , Timeout timeout_

    , TX_done_callback    tx_done_callback
    , RX_done_callback    rx_done_callback
    , TX_timeout_callback tx_timeout_callback
    , RX_timeout_callback rx_timeout_callback
    , RX_error_callback   rx_error_callback
) : tx_done_callback    {tx_done_callback.value}
  , rx_done_callback    {rx_done_callback.value}
  , tx_timeout_callback {tx_timeout_callback.value}
  , rx_timeout_callback {rx_timeout_callback.value}
  , rx_error_callback   {tx_timeout_callback.value}
  , spi  {spi.value}
  , mosi {mosi.value}
  , miso {miso.value}
  , clk  {clk.value}
  , frequency {frequency}
  , power {power_.value}
  , bandwidth {bandwidth}
  , spreading_factor {spreading_factor}
  , coding_rate {coding_rate}
  , preambula_length {preambula_length_.value}
  , fix_length_payload {fix_length_payload_.value}
  , iq_inversion {iq_inversion_.value}
  , symbol_timeout {symbol_timeout_.value}
  , timeout {timeout_.value}


{
    pointer = this;
    board_init();

    events.TxDone    = ::tx_done_callback;
    events.RxDone    = ::rx_done_callback;
    events.TxTimeout = ::tx_timeout_callback;
    events.RxTimeout = ::rx_timeout_callback;
    events.RxError   = ::rx_error_callback;
    radio.Init (&events);

    radio.SetChannel (static_cast<uint32_t>(frequency));
    radio.SetTxConfig (
          MODEM_LORA
        , power
        , 0
        , static_cast<uint32_t>(bandwidth)
        , static_cast<uint32_t>(spreading_factor)
        , static_cast<uint8_t> (coding_rate)
        , preambula_length
        , fix_length_payload
        , true
        , 0
        , 0
        , iq_inversion
        , 3000 
    );
    radio.SetRxConfig (
          MODEM_LORA
        , static_cast<uint32_t>(bandwidth)
        , static_cast<uint32_t>(spreading_factor)
        , static_cast<uint8_t> (coding_rate)
        , 0
        , preambula_length
        , symbol_timeout
        , fix_length_payload
        , 0
        , true
        , 0
        , 0
        , iq_inversion
        , true 
    );
}

void Radio::board_init() // BoardInitMcu( );
{
    if (not is_init) {
        HAL_Init( );
        // LEDs
        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        SystemClockConfig( );
        RtcInit( );
        BoardUnusedIoInit( );
    } else {
        SystemClockReConfig( );
    }

#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    SpiInit( &SX126x.Spi, spi, mosi, miso, clk, NC );
    SX126xIoInit( );
#elif defined( SX1272MB2DAS)
    SpiInit( &SX1272.Spi, spi, mosi, miso, sclk, NC );
    SX1272IoInit( );
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    SpiInit( &SX1276.Spi, spi, mosi, miso, clk, NC );
    SX1276IoInit( );
#endif

    is_init = true;
}

void Radio::sleep() {radio.Sleep();}
void Radio::receive() { radio.Rx (timeout); }

} // namespace lmn {

extern "C" {
void tx_done_callback()
{
    lmn::Radio::pointer->sleep();
    lmn::Radio::pointer->tx_done_callback();
}
void rx_done_callback( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    lmn::Radio::pointer->sleep();
    lmn::Radio::pointer->rx_done_callback(payload,size,rssi,snr);
}
void tx_timeout_callback() 
{ 
    lmn::Radio::pointer->sleep();
    lmn::Radio::pointer->tx_timeout_callback();
}
void rx_timeout_callback() 
{
    lmn::Radio::pointer->sleep();
    lmn::Radio::pointer->rx_timeout_callback();
}
void rx_error_callback() 
{
    lmn::Radio::pointer->sleep();
    lmn::Radio::pointer->rx_error_callback();
}


void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
}

void BoardDeInitMcu( void )
{
#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    SpiDeInit( &SX126x.Spi );
    SX126xIoDeInit( );
#elif defined( SX1272MB2DAS)
    SpiDeInit( &SX1272.Spi );
    SX1272IoDeInit( );
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );
#endif
}

void BoardUnusedIoInit( void ) // static
{
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
}

void SystemClockConfig( void )
{
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
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    // Enable HSI
    __HAL_RCC_HSI_ENABLE( );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    // Enable PLL
    __HAL_RCC_PLL_ENABLE( );

    // Wait till PLL is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    // Select PLL as system clock source
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    // Wait till PLL is used as system clock source
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}




} // extern "C" {