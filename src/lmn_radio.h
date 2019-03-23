#pragma once

// c++
#include <cstddef>
#include "../mculib3/src/function.h"
#include <cstdint> // move to literals
#include "../mculib3/src/literals.h"

constexpr uint32_t operator "" _dBm (unsigned long long val) { return val; }

extern "C" {
#include "../LoRaMac-node/src/radio/radio.h"
// #include "../LoRaMac-node/src/boards/pinName-board.h"
// #include "../LoRaMac-node/src/boards/pinName-ioe.h"
// #include "../LoRaMac-node/src/system/gpio.h"
// #include "../LoRaMac-node/src/system/spi.h"

void BoardLowPowerHandler();
} // extern "C" {


namespace lmn {

enum SpiId_t { SPI_1, SPI_2 };
enum PinNames {
    PA_0 = 0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7, PA_8, PA_9, PA_10, PA_11, PA_12, PA_13, PA_14, PA_15,
    PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7, PB_8, PB_9, PB_10, PB_11, PB_12, PB_13, PB_14, PB_15,    
    PC_0, PC_1, PC_2, PC_3, PC_4, PC_5, PC_6, PC_7, PC_8, PC_9, PC_10, PC_11, PC_12, PC_13, PC_14, PC_15,    
    PD_0, PD_1, PD_2, PD_3, PD_4, PD_5, PD_6, PD_7, PD_8, PD_9, PD_10, PD_11, PD_12, PD_13, PD_14, PD_15,    
    PE_0, PE_1, PE_2, PE_3, PE_4, PE_5, PE_6, PE_7, PE_8, PE_9, PE_10, PE_11, PE_12, PE_13, PE_14, PE_15,    
    PF_0, PF_1, PF_2, PF_3, PF_4, PF_5, PF_6, PF_7, PF_8, PF_9, PF_10, PF_11, PF_12, PF_13, PF_14, PF_15,    
    PH_0, PH_1, PH_2, PH_3, PH_4, PH_5, PH_6, PH_7, PH_8, PH_9, PH_10, PH_11, PH_12, PH_13, PH_14, PH_15

};

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
// void rx_done_callback(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr)
using RX_done_callback    = Construct_wrapper<Callback<uint8_t*,uint16_t,int16_t,int8_t>>;
using TX_timeout_callback = Construct_wrapper<Callback<>, 1>;
using RX_timeout_callback = Construct_wrapper<Callback<>, 2>;
using RX_error_callback   = Construct_wrapper<Callback<>, 3>;

class Radio;

struct C_wrapper {
    Radio* radio {nullptr};
};

class Radio {
public:
    Radio (
          SPI                spi
        , MOSI               mosi
        , MISO               miso
        , CLK                clk
        , Region_frequency   frequency
        , Power              power_
        , Bandwidth          bandwidth
        , Spreading_factor   spreading_factor
        , Coding_rate        coding_rate
        , Preambula_length   preambula_length_
        , Fix_length_payload fix_length_payload_
        , IQ_inversion       iq_inversion_
        , Symbol_timeout     symbol_timeout_
        , Timeout            timeout_

        , TX_done_callback    tx_done_callback
        , RX_done_callback    rx_done_callback
        , TX_timeout_callback tx_timeout_callback
        , RX_timeout_callback rx_timeout_callback
        , RX_error_callback   rx_error_callback
    );

    
    void sleep();
    void receive();

    static Radio*          pointer;
    Callback<>             tx_done_callback;
    RX_done_callback::type rx_done_callback;
    Callback<>             tx_timeout_callback;
    Callback<>             rx_timeout_callback;
    Callback<>             rx_error_callback;


private:
    const SpiId_t          spi;
    const PinNames         mosi;
    const PinNames         miso;
    const PinNames         clk;
    const Region_frequency frequency;
    const uint8_t          power;
    const Bandwidth        bandwidth;
    const Spreading_factor spreading_factor;
    const Coding_rate      coding_rate;
    const uint16_t         preambula_length;
    const bool             fix_length_payload;
    const bool             iq_inversion;
    const uint16_t         symbol_timeout;
    const uint32_t         timeout;
    bool                   is_init {false};
    const Radio_s&         radio {::Radio};
    RadioEvents_t          events;

    void board_init(); // BoardInitMcu( );
};


} // namespace lmn {


