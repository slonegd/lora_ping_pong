
// #include "lmn_radio.h"

// #include <string.h>
// // #include <string_view>

#include <string.h>

#include "lmn_radio.h"

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";
// const std::string_view PingMsg = "PING";
// const std::string_view PongMsg = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int8_t RssiValue = 0;
int8_t SnrValue = 0;

extern Gpio_t Led1;
extern Gpio_t Led2;

int main( void )
{
    bool isMaster = true;

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
          lmn::SPI       {SPI_1}
        , lmn::MOSI      {PA_7}
        , lmn::MISO      {PA_6}
        , lmn::CLK       {PA_5}
        , lmn::Region_frequency::RU864
        , lmn::Power     {14_dBm}
        , lmn::Bandwidth::_500_kHz
        , lmn::Spreading_factor::_7
        , lmn::Coding_rate::_4_5
        , lmn::Preambula_length {8}
        , lmn::Fix_length_payload {false}
        , lmn::IQ_inversion {false}
        , lmn::Symbol_timeout {0}

        , lmn::TX_done_callback {[&]{ 
            Radio.Sleep( );
            state = TX;
        }}
        , lmn::RX_done_callback {[&](uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
            Radio.Sleep( );
            BufferSize = size;
            memcpy( Buffer, payload, BufferSize );
            RssiValue = rssi;
            SnrValue = snr;
            state = RX;
        }}
        , lmn::TX_timeout_callback {[&]{
            Radio.Sleep( );
            state = TX_TIMEOUT;
        }}
        , lmn::RX_timeout_callback {[&]{
            Radio.Sleep( );
            state = RX_TIMEOUT;
        }}
        , lmn::RX_error_callback {[&]{
            Radio.Sleep( );
            state = RX_ERROR;
        }} 
    );

    Radio.Rx( RX_TIMEOUT_VALUE );

    while( 1 )
    {
        switch( state )
        {
        case RX:
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PONG
                        GpioToggle( &Led1 );

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( auto i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                  //   else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        GpioToggle( &Led2 ); // Set LED off
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                  //   if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PING
                        GpioToggle( &Led1 );

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( auto i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            state = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            GpioToggle( &Led2 );
            Radio.Rx( RX_TIMEOUT_VALUE );
            state = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( auto i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                DelayMs( 1 );
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            state = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
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
    }
}

