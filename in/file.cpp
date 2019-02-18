#include "file.h"

#if not defined(PIN)
   #define PIN PA11
#endif

auto& pin = Pin::make<mcu::PIN,PinMode::Output>();

void toggle_pin()
{
   pin.toggle();
}