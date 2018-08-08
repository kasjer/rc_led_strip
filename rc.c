#include <iostm8s103f3.h>
#include <intrinsics.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dispatcher.h"
#include "rc.h"

uint8_t bit_end_h;
uint8_t bit_end_l;
uint8_t hi_end_h;
uint8_t hi_end_l;

uint8_t rc_buf[8];
uint8_t rc_bits;

#pragma vector = TIM1_CAPCOM_CC1IF_vector
__interrupt void TIM1_CAPCOM_CCxIF_ISR(void)
{
//    PD_ODR_ODR2 = 1;
//    PD_ODR_ODR2 = 0;
    if (TIM1_SR1_CC1IF)
    {
        bit_end_h = TIM1_CCR1H;
        bit_end_l = TIM1_CCR1L;
        ADD_EVENT(EVENT_BIT_END);
        TIM1_IER_CC1IE = 0;
        TIM1_SR1_UIF = 0;
        TIM1_IER_UIE = 1;
    }
    if (TIM1_SR1_CC2IF)
    {
        hi_end_h = TIM1_CCR2H;
        hi_end_l = TIM1_CCR2L;
        ADD_EVENT(EVENT_HI_END);
    }
}

#pragma vector = TIM1_OVR_UIF_vector
__interrupt void TIM1_OVR_UIF_ISR(void)
{
//    PD_ODR_ODR3 = 1;
//    PD_ODR_ODR3 = 0;
    TIM1_SR1_UIF = 0;
    if (TIM1_SR1_CC1IF)
    {
        bit_end_h = TIM1_CCR1H;
        bit_end_l = TIM1_CCR1L;
        ADD_EVENT(EVENT_BIT_END);
    }
    else
    {
        ADD_EVENT(EVENT_DATA_END);
        TIM1_IER_UIE = 0;
        TIM1_IER_CC1IE = 1;
    }
}

void init_rc(void)
{
    TIM1_PSCRH = 0x6;
    TIM1_PSCRL = 0x40;
    TIM1_ARRH = 0x00;
    TIM1_ARRL = 200; // 20 ms
    TIM1_CCMR1_CC1S = 1;
    TIM1_CCER1_CC1P = 1;
    TIM1_CCMR2_CC2S = 2;
    TIM1_CCER1_CC2P = 0;
    TIM1_SMCR_TS = 5;
    TIM1_SMCR_SMS = 4;
    TIM1_CCER1_CC1E = 1;
    TIM1_CCER1_CC2E = 1;
    TIM1_IER_UIE = 0;
    TIM1_IER_CC1IE = 1;
    TIM1_IER_CC2IE = 1;
    TIM1_CR1_CEN = 1;
}

void __weak rc_default(uint8_t len, const uint8_t *data)
{
}

uint8_t rc_handle_event(uint8_t e)
{
    if (e == EVENT_BIT_END)
    {
        if (bit_end_h != 0 || hi_end_h != 0)
        {
            rc_bits = 0;
        }
        else if (bit_end_l > 24 || bit_end_l < 8 ||
                 hi_end_l < 3 || hi_end_l > 6)
        {
            rc_bits = 0;
        }
        else
        {
            if (hi_end_l + hi_end_l + hi_end_l < bit_end_l)
            {
                rc_buf[rc_bits >> 3] |= (1 << (rc_bits & 7));
            }
            else
            {
                rc_buf[rc_bits >> 3] &= ~(1 << (rc_bits & 7));
            }
            rc_bits++;
        }
    }
    else if (e == EVENT_HI_END)
    {
    }
    else if (e == EVENT_DATA_END)
    {
        uint8_t bytes = rc_bits >> 3;

        if (bytes > 0)
        {
            struct rc_code *rc_code = (struct rc_code *)rc_codes;

            while (rc_code->len)
            {
                if (bytes == rc_code->len &&
                    memcmp(rc_code->code, rc_buf, rc_code->len) == 0)
                {
                    rc_key_pressed(rc_code->key_code);
                    return 1;
                }
                rc_code = (struct rc_code *)((uint8_t *)rc_code + 2 + rc_code->len);
            }
            rc_default(bytes, rc_buf);
//            for (uint8_t i = 0; i != bytes; ++i)
//            {
//                printf("0x%02X ", rc_buf[i]);
//            }
//            puts("");
//                fwrite(rc_buf, rc_bits >> 3, 1, stdout);
//                fflush(stdout);
        }
        rc_bits = 0;
    }
    else
        return 0;
    return 1;
}
