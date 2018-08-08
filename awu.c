#include <iostm8s103f3.h>
#include <intrinsics.h>
#include <stdint.h>
#include "dispatcher.h"
#include "awu.h"

static uint8_t awu_timeout;
static uint8_t awu_timeout_event;

#pragma vector = AWU_vector
__interrupt void awu_vector_isr(void)
{
    // I think that this would read register to clear interrupt ????
    uint8_t v = AWU_CSR1;
    ADD_EVENT(EVENT_AWU);
}

uint8_t awu_handle_event(uint8_t e)
{
    if (e == EVENT_AWU)
    {
        if (awu_timeout)
        {
            awu_timeout--;
            if (awu_timeout == 0)
                ADD_EVENT(awu_timeout_event);
        }
        return 1;
    }
    else
        return 0;
}

void awu_set(uint8_t apr, uint8_t tbr, uint8_t csr)
{
    AWU_APR = apr;
    AWU_TBR = tbr;
    AWU_CSR1 = csr;
}

void set_awu_timeout(uint8_t seconds, uint8_t event)
{
    if (seconds)
    {
        awu_timeout = seconds;
        awu_timeout_event = event;
        awu_set(62, 12, MASK_AWU_CSR1_AWUEN);
    }
    else
    {
        awu_set(62, 0, 0);
    }
}
