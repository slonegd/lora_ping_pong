#define STM32F030x6
#define F_OSC   8000000UL
#define F_CPU   48000000UL
#include "periph_rcc.h"
#include "flash.h"
#include "pin.h"
#include "timers.h"
#include "periph_dma.h"
#include "file.h"


/// эта функция вызываеться первой в startup файле
extern "C" void init_clock ()
{
   // FLASH::set (FLASH::Latency::_1);

   mcu::make_reference<mcu::Periph::RCC>()
      .set (mcu::RCC:: AHBprescaler::AHBnotdiv)
      .set (mcu::RCC:: APBprescaler::APBnotdiv)
      .set (mcu::RCC::  SystemClock::CS_PLL)
      .set (mcu::RCC::    PLLsource::HSIdiv2)
      .set (mcu::RCC::PLLmultiplier::_12)
      .on_PLL()
      .wait_PLL_ready();
}



int main()
{
   auto& dma = mcu::make_reference<mcu::Periph::DMA1>();
   dma.clear_interrupt_flags(dma.Channel::_1);
   dma.is_transfer_complete_interrupt (dma.Channel::_1);

   auto& pc8 = Pin::make<mcu::PC8, mcu::PinMode::Output>();
   Timer timer {200};


   while(1) {

      if (timer.event()) {
         pc8 ^= 1;
      }
      toggle_pin();

   } // while(1) {

}
