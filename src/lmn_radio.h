#pragma once

// c++
#include "../mculib3/src/function.h"
#include <cstdint> // move to literals
#include "../mculib3/src/literals.h"

constexpr uint32_t operator "" _dBm (unsigned long long val) { return val; }


extern "C" {
// from board
#include "stm32l1xx.h"
#include "gpio.h"
#include "spi.h"
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

/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;


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

static void BoardUnusedIoInit( void )
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

// с++
void set_callbacks();
} // extern "C" {




namespace lmn {

struct Radio;

struct C_wrappwer {
    Radio* radio {nullptr};
} c_wrapper;

enum class Region_frequency : uint32_t {
    AS923 = 923_MHz,
    AU915 = 915_MHz,
    CN470 = 470_MHz,
    CN779 = 779_MHz,
    EU433 = 433_MHz,
    EU868 = 868_MHz,
    KR920 = 920_MHz,
    IN865 = 865_MHz,
    US915 = 915_MHz,
    RU864 = 864_MHz,
};

enum class Bandwidth : uint8_t { _125_kHz = 0, _250_kHz, _500_kHz };
enum class Spreading_factor { _7 = 7, _8, _9, _10, _11, _12 };
enum class Coding_rate { _4_5 = 1, _4_6, _4_7, _4_8 };

template<class T, size_t n = 0> // n for unique with same T
struct Construct_wrapper {
    using type = T;
    T value;
    explicit Construct_wrapper (T value) : value{value} {}
};
using SPI       = Construct_wrapper<SpiId_t>;
using MOSI      = Construct_wrapper<PinNames>;
using MISO      = Construct_wrapper<PinNames, 1>;
using CLK       = Construct_wrapper<PinNames, 2>;
using Power     = Construct_wrapper<uint8_t>;
using Preambula_length   = Construct_wrapper<uint16_t>;
using Fix_length_payload = Construct_wrapper<bool>;
using IQ_inversion       = Construct_wrapper<bool, 1>;
using Symbol_timeout     = Construct_wrapper<uint16_t, 1>;
using Timeout            = Construct_wrapper<uint32_t>;


template<class...Args>
using Callback = Function<void(Args...)>;

using TX_done_callback    = Construct_wrapper<Callback<>>;
using RX_done_callback    = Construct_wrapper<Callback<uint8_t*,uint16_t,int16_t,int8_t>>;
using TX_timeout_callback = Construct_wrapper<Callback<>, 1>;
using RX_timeout_callback = Construct_wrapper<Callback<>, 2>;
using RX_error_callback   = Construct_wrapper<Callback<>, 3>;



class Radio {
    const SpiId_t  spi;
    const PinNames mosi;
    const PinNames miso;
    const PinNames clk;
    const Region_frequency frequency;
    const uint8_t power;
    const Bandwidth bandwidth;
    const Spreading_factor spreading_factor;
    const Coding_rate coding_rate;
    const uint16_t preambula_length;
    const bool fix_length_payload;
    const bool iq_inversion;
    const uint16_t symbol_timeout;
    const uint32_t timeout;
    bool is_init {false};
    
    const Radio_s& radio {::Radio};
public:
    Callback<> tx_done_callback;
    RX_done_callback::type rx_done_callback;
    Callback<> tx_timeout_callback;
    Callback<> rx_timeout_callback;
    Callback<> rx_error_callback;

    Radio (
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
    ) : spi  {spi.value}
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

      , tx_done_callback    {tx_done_callback.value}
      , rx_done_callback    {rx_done_callback.value}
      , tx_timeout_callback {tx_timeout_callback.value}
      , rx_timeout_callback {rx_timeout_callback.value}
      , rx_error_callback   {tx_timeout_callback.value}
    {
        c_wrapper.radio = this;
        board_init();
        set_callbacks();
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

    void board_init() // BoardInitMcu( );
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

    void sleep() {radio.Sleep();}
    void receive() { radio.Rx (timeout); }
};


} // namespace lmn {


extern "C" {
void BoardInitMcu() { lmn::c_wrapper.radio->board_init(); }
void OnTxDone_()
{
    lmn::c_wrapper.radio->sleep();
    lmn::c_wrapper.radio->tx_done_callback();
}
void OnRxDone_( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    lmn::c_wrapper.radio->sleep();
    lmn::c_wrapper.radio->rx_done_callback(payload,size,rssi,snr);
}
void OnTxTimeout_() 
{ 
    lmn::c_wrapper.radio->sleep();
    lmn::c_wrapper.radio->tx_timeout_callback();
}
void OnRxTimeout_() 
{
    lmn::c_wrapper.radio->sleep();
    lmn::c_wrapper.radio->rx_timeout_callback();
}
void OnRxError_() 
{
    lmn::c_wrapper.radio->sleep();
    lmn::c_wrapper.radio->rx_error_callback();
}

RadioEvents_t RadioEvents;
void set_callbacks()
{
    RadioEvents.TxDone = OnTxDone_;
    RadioEvents.RxDone = OnRxDone_;
    RadioEvents.TxTimeout = OnTxTimeout_;
    RadioEvents.RxTimeout = OnRxTimeout_;
    RadioEvents.RxError = OnRxError_;

    Radio.Init( &RadioEvents );
}
} // extern "C" {