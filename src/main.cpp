#include <string.h>
#include <array>
#include <string_view>
#include <cstddef>

#include "lmn_radio.h"

extern "C" {
    #include "delay.h" // delete then
}

extern Gpio_t Led1;
extern Gpio_t Led2;

int main( void )
{
    bool is_master {true};
    bool is_receive {false};
    constexpr std::string_view ping_message = "PING";
    constexpr std::string_view pong_message = "PONG";
    std::array<char, 64> buffer; 

    enum State {
        LOWPOWER,
        RX,
        RX_TIMEOUT,
        RX_ERROR,
        TX,
        TX_TIMEOUT,
    } state {LOWPOWER};

    // Target board initialization
    auto radio = lmn::Radio (
          lmn::SPI                {SPI_1}
        , lmn::MOSI               {PA_7}
        , lmn::MISO               {PA_6}
        , lmn::CLK                {PA_5}
        , lmn::Region_frequency   ::RU864
        , lmn::Power              {14_dBm}
        , lmn::Bandwidth          ::_500_kHz
        , lmn::Spreading_factor   ::_7
        , lmn::Coding_rate        ::_4_5
        , lmn::Preambula_length   {8}
        , lmn::Fix_length_payload {false}
        , lmn::IQ_inversion       {false}
        , lmn::Symbol_timeout     {0}
        , lmn::Timeout            {1_s}

        , lmn::TX_done_callback {[&]{ state = TX; }}

        , lmn::RX_done_callback {[&](uint8_t *payload, uint16_t size, int16_t, int8_t){
            is_receive = size > 0;
            std::copy (payload, payload + size, buffer.begin());
            state = RX;
        }}

        , lmn::TX_timeout_callback {[&]{ state = TX_TIMEOUT; }}

        , lmn::RX_timeout_callback {[&]{ state = RX_TIMEOUT; }}

        , lmn::RX_error_callback   {[&]{ state = RX_ERROR; }} 
    );

    radio.receive();

    while (1) {
        switch( state ) {
        case RX:
             if (is_receive) {
                is_receive = false;
                std::string_view message (buffer.data(), pong_message.size());
                bool right_message = is_master ? message == pong_message : message == ping_message;
                if (right_message) {
                    GpioToggle (&Led1);
                    DelayMs (1);
                    auto& answer = is_master ? ping_message : pong_message;
                    std::copy (answer.begin(), answer.end(), buffer.begin());
                    Radio.Send( (uint8_t*)buffer.data(), buffer.size() );
                } else {
                    is_master ^= 1;
                    radio.receive();
                }
            }
            state = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            GpioToggle( &Led2 );
            radio.receive();
            state = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if (is_master) {
                // Send the next PING frame
                std::copy (ping_message.begin(), ping_message.end(), buffer.begin());
                Radio.Send( (uint8_t*)buffer.data(), buffer.size() );
            } else {
                radio.receive();
            }
            state = LOWPOWER;
            break;
        case TX_TIMEOUT:
            radio.receive();
            state = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        BoardLowPowerHandler( );
        // Process Radio IRQ
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }
    } // while (1) {
}




