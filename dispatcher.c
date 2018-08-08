#include <intrinsics.h>
#include <stdint.h>
#include "dispatcher.h"

uint8_t events[8];
uint8_t event_count;
uint8_t event_ptr;

void __noreturn event_loop(void)
{
    while (1)
    {
        uint8_t e = 0;
        if (event_count)
        {
            uint8_t i;
            __disable_interrupt();
            i = (event_ptr - event_count) & 7;
            e = events[i];
            event_count--;
            __enable_interrupt();
        }
        if (e)
        {
            for (uint8_t i = 0; event_handlers[i].fn; ++i)
            {
                if (event_handlers[i].fn(e))
                    break;
            }
        }
        else
            ;//__wait_for_interrupt();
//        __no_operation();
    }
}
