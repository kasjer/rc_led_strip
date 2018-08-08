#include <iostm8s103f3.h>
#include <intrinsics.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "dispatcher.h"
#include "rc.h"
#include "awu.h"

#define HSI_CLOCK   16000000
#define CPU_CLOCK   16000000

#ifndef NDEBUG
#define BAUDRATE_SUPPORT_9600
#define BAUDRATE_SUPPORT_115200

uint8_t txbuf[8];
volatile int8_t tx1;
volatile int8_t tx2;

#pragma vector = UART1_T_TXE_vector
__interrupt void UART1_TXE_Isr(void)
{
    int8_t tmp = tx2;
    if (tx1 == tmp)
    {
        UART1_CR2_TIEN = 0;
        PB_ODR_bit.ODR5 = 1;
    }
    else
    {
        UART1_DR = txbuf[tmp & 7];
        PB_ODR_bit.ODR5 = 0;
        ++tmp;
        tx2 = tmp & 15;
    }
}

typedef enum
{
#ifdef BAUDRATE_SUPPORT_9600
    BAURATE_9600,
#endif
#ifdef BAUDRATE_SUPPORT_19200
    BAURATE_19200,
#endif
#ifdef BAUDRATE_SUPPORT_57600
    BAURATE_57600,
#endif
#ifdef BAUDRATE_SUPPORT_115200
    BAURATE_115200,
#endif
#ifdef BAUDRATE_SUPPORT_1000000
    BAURATE_1000000,
#endif
    BAUDRATE_SUPPORTED_COUNT
} baud_rate_t;

typedef struct
{
    uint8_t brr1;
    uint8_t brr2;
} brr_t;

#define BRR_DIV(clk, br) (((clk) + (br) - 1) / (br))
#define BRR1_VALUE(clk, br) (BRR_DIV((clk), (br)) >> 4)
#define BRR2_VALUE(clk, br) ((BRR_DIV((clk), (br)) & 0xF) | ((BRR_DIV((clk), (br)) >> 8)))

const brr_t uart_dividers_for_clock[BAUDRATE_SUPPORTED_COUNT][8] = {
#ifdef BAUDRATE_SUPPORT_9600
    {
        { BRR1_VALUE(HSI_CLOCK / 1, 9600), BRR2_VALUE(HSI_CLOCK / 1, 9600) },
        { BRR1_VALUE(HSI_CLOCK / 2, 9600), BRR2_VALUE(HSI_CLOCK / 2, 9600) },
        { BRR1_VALUE(HSI_CLOCK / 4, 9600), BRR2_VALUE(HSI_CLOCK / 4, 9600) },
        { BRR1_VALUE(HSI_CLOCK / 8, 9600), BRR2_VALUE(HSI_CLOCK / 8, 9600) },
    },
#endif
#ifdef BAUDRATE_SUPPORT_19200
    {
        { BRR1_VALUE(HSI_CLOCK / 1, 19200), BRR2_VALUE(HSI_CLOCK / 1, 19200) },
        { BRR1_VALUE(HSI_CLOCK / 2, 19200), BRR2_VALUE(HSI_CLOCK / 2, 19200) },
        { BRR1_VALUE(HSI_CLOCK / 4, 19200), BRR2_VALUE(HSI_CLOCK / 4, 19200) },
        { BRR1_VALUE(HSI_CLOCK / 8, 19200), BRR2_VALUE(HSI_CLOCK / 8, 19200) },
    },
#endif
#ifdef BAUDRATE_SUPPORT_57600
    {
        { BRR1_VALUE(HSI_CLOCK / 1, 57600), BRR2_VALUE(HSI_CLOCK / 1, 57600) },
        { BRR1_VALUE(HSI_CLOCK / 2, 57600), BRR2_VALUE(HSI_CLOCK / 2, 57600) },
        { BRR1_VALUE(HSI_CLOCK / 4, 57600), BRR2_VALUE(HSI_CLOCK / 4, 57600) },
        { BRR1_VALUE(HSI_CLOCK / 8, 57600), BRR2_VALUE(HSI_CLOCK / 8, 57600) },
    },
#endif
#ifdef BAUDRATE_SUPPORT_115200
    {
        { BRR1_VALUE(HSI_CLOCK / 1, 115200), BRR2_VALUE(HSI_CLOCK / 1, 115200) },
        { BRR1_VALUE(HSI_CLOCK / 2, 115200), BRR2_VALUE(HSI_CLOCK / 2, 115200) },
        { BRR1_VALUE(HSI_CLOCK / 4, 115200), BRR2_VALUE(HSI_CLOCK / 4, 115200) },
        { BRR1_VALUE(HSI_CLOCK / 8, 115200), BRR2_VALUE(HSI_CLOCK / 8, 115200) },
    },
#endif
#ifdef BAUDRATE_SUPPORT_1000000
    {
        { BRR1_VALUE(HSI_CLOCK / 1, 1000000), BRR2_VALUE(HSI_CLOCK / 1, 1000000) },
        { BRR1_VALUE(HSI_CLOCK / 2, 1000000), BRR2_VALUE(HSI_CLOCK / 2, 1000000) },
        { BRR1_VALUE(HSI_CLOCK / 4, 1000000), BRR2_VALUE(HSI_CLOCK / 4, 1000000) },
        { BRR1_VALUE(HSI_CLOCK / 8, 1000000), BRR2_VALUE(HSI_CLOCK / 8, 1000000) },
    },
#endif
};
#if MASTER_CLOCK_ALLOW_16000000
#endif
#if MASTER_CLOCK_ALLOW_8000000
#endif
#if MASTER_CLOCK_ALLOW_4000000
#endif
#if MASTER_CLOCK_ALLOW_2000000
#endif

void set_baudrate(baud_rate_t br)
{
    UART1_BRR2 = uart_dividers_for_clock[br][CLK_CKDIVR_HSIDIV].brr2;
    UART1_BRR1 = uart_dividers_for_clock[br][CLK_CKDIVR_HSIDIV].brr1;
}

size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    size_t i;
    uint8_t tmp = tx1;
    for (i = 0; i < bufSize; ++i)
    {
        // If output buffer if full wait till free space shows up
        while (abs(tmp - tx2) == 8)
        {
            UART1_CR2_TIEN = 1;
            __wait_for_interrupt();
        }
        // put char in output buffer, for interrupt to use
        txbuf[tmp & 7] = *buf++;
        tmp = (tmp + 1) & 15;
        tx1 = tmp;
        UART1_CR2_TIEN = 1;
    }
    return bufSize;
}

#endif // NDEBUG

__no_init uint8_t eeprom_data[5] @ ".eeprom.noinit";

uint8_t brightness;
uint8_t power_mask;
uint8_t led_mask[3];

static void restore_eeprom_values(void)
{
    brightness = eeprom_data[0];
    power_mask = eeprom_data[1] ? 0xFF : 0;
    led_mask[0] = eeprom_data[2];
    led_mask[1] = eeprom_data[3];
    led_mask[2] = eeprom_data[4];
}

static uint8_t eeprom_write_needed(void)
{
    return (eeprom_data[0] != brightness) ||
       (eeprom_data[1] != power_mask) ||
       (eeprom_data[2] != led_mask[0]) ||
       (eeprom_data[3] != led_mask[1]) ||
       (eeprom_data[4] != led_mask[2]);
}

static void store_eeprom_values(void)
{
    if (FLASH_IAPSR_DUL == 0)
    {
        FLASH_DUKR = 0xAE;
        FLASH_DUKR = 0x56;
    }
    if (eeprom_data[0] != brightness)
        eeprom_data[0] = brightness;
    if (eeprom_data[1] != power_mask)
        eeprom_data[1] = power_mask;
    if (eeprom_data[2] != led_mask[0])
        eeprom_data[2] = led_mask[0];
    if (eeprom_data[3] != led_mask[1])
        eeprom_data[3] = led_mask[1];
    if (eeprom_data[4] != led_mask[2])
        eeprom_data[4] = led_mask[2];
    FLASH_IAPSR_DUL = 0;
}

#define EVENT_10MS_TICK 20

#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_Isr(void)
{
    ADD_EVENT(EVENT_10MS_TICK);
    // Clear interrupt
    TIM2_SR1_UIF = 0;
}


//static const uint8_t bm[] = { 0, 10, 20, 30, 50, 75, 100, 128, 152, 180, 205, 230, 255 };
static const uint8_t bm[] = { 0, 6, 12, 18, 30, 45, 60, 78, 92, 109, 124, 140, 156 };
static const uint8_t digi[10] = { 0, 1, 2, 4, 6, 7, 8, 9, 10, 12 };

void init_tick(void)
{
    TIM2_CR1_CEN = 0;
    TIM2_CR1_ARPE = 0;
    TIM2_CR1_URS = 0;
    TIM2_PSCR = 10;
    TIM2_ARRH = 0;
    TIM2_ARRL = 156; // count to 156 ~10ms period at PSC 10
    TIM2_CR1_OPM = 0;    // Continues mode = 0, One shot = 1
    TIM2_SR1_UIF = 0;
    TIM2_IER_UIE = 1;
    TIM2_CR1_CEN = 1;
}

void pwm(uint8_t r, uint8_t g, uint8_t b)
{
    // Red
    TIM2_CCMR1_OC1M = 7; // PWM = 7, 4 - force low
    TIM2_CCMR1_OC1PE = 0; // Preload 1 - enabled, 0 - disable
    TIM2_CCMR1_CC1S = 0;  // CC1 output
    TIM2_CCER1_CC1P = 1;  // CC1 output polarity
    TIM2_CCER1_CC1E = 1;  // CC1 output enabled
    TIM2_CCR1H = 0;
    TIM2_CCR1L = r; // switch at on_time

    // Green
    TIM2_CCMR2_OC2M = 7; // PWM = 7, 4 - force low
    TIM2_CCMR2_OC2PE = 0; // Preload 1 - enabled, 0 - disable
    TIM2_CCMR2_CC2S = 0;  // CC1 output
    TIM2_CCER1_CC2P = 1;  // CC1 output polarity
    TIM2_CCER1_CC2E = 1;  // CC1 output enabled
    TIM2_CCR2H = 0;
    TIM2_CCR2L = g; // switch at on_time

    // Blue
    TIM2_CCMR3_OC3M = 7; // PWM = 7, 4 - force low
    TIM2_CCMR3_OC3PE = 0; // Preload 1 - enabled, 0 - disable
    TIM2_CCMR3_CC3S = 0;  // CC1 output
    TIM2_CCER2_CC3P = 1;  // CC1 output polarity
    TIM2_CCER2_CC3E = 1;  // CC1 output enabled
    TIM2_CCR3H = 0;
    TIM2_CCR3L = b; // switch at on_time
    TIM2_CR1_CEN = 1;
}

void key(void *arg)
{
#ifndef NDEBUG
    puts((char *)arg);
#endif
}

#define EVENT_EEPROM_WRITE 0x20

static uint8_t delayed_write_handle_event(uint8_t event)
{
    if (event != EVENT_EEPROM_WRITE)
        return 0;
    store_eeprom_values();
    return 1;
}

static void delayed_eeprom_write(void)
{
    if (eeprom_write_needed())
        set_awu_timeout(5, EVENT_EEPROM_WRITE);
}

static void update_leds(void)
{
    pwm(bm[brightness] & led_mask[0] & power_mask,
        bm[brightness] & led_mask[1] & power_mask,
        bm[brightness] & led_mask[2] & power_mask);
    delayed_eeprom_write();
}

void down(void)
{
#ifndef NDEBUG
    puts("down");
#endif
    if (brightness)
    {
        brightness--;
        update_leds();
    }
}

void up(void)
{
#ifndef NDEBUG
    puts("up");
#endif
    if (brightness < sizeof(bm) - 1)
    {
        brightness++;
        update_leds();
    }
}

void yellow(void)
{
#ifndef NDEBUG
    puts("yellow");
#endif
    power_mask = 0xFF;
    led_mask[0] = led_mask[1] = 255;
    led_mask[2] = 255;
    update_leds();
}

void rc_default(uint8_t len, const uint8_t *rc_buf)
{
#ifndef NDEBUG
    printf("RC_CODE%d( ", len);
    for (uint8_t i = 0; i != len; ++i)
    {
        printf("0x%02X, ", rc_buf[i]);
    }
    puts("KEY_ ),");
#endif
}

void red(void)
{
#ifndef NDEBUG
    puts("red");
#endif
    power_mask = 0xFF;
    led_mask[0] = 255;
    led_mask[1] = 0;
    led_mask[2] = 0;
    update_leds();
}

void green(void)
{
#ifndef NDEBUG
    puts("green");
#endif
    power_mask = 0xFF;
    led_mask[0] = 0;
    led_mask[1] = 255;
    led_mask[2] = 0;
    update_leds();
}

void blue(void)
{
#ifndef NDEBUG
    puts("blue");
#endif
    power_mask = 0xFF;
    led_mask[0] = 0;
    led_mask[1] = 0;
    led_mask[2] = 255;
    update_leds();
}

void power(void)
{
#ifndef NDEBUG
    puts("power");
#endif
    power_mask ^= 255;
    update_leds();
}

void digit(uint8_t d)
{
    brightness = digi[d];
    power_mask = 0xFF;
#ifndef NDEBUG
    d += '0';
    putchar(d);
    puts("");
#endif
    update_leds();
}

enum keys
{
    KEY_POWER,
    KEY_UP,
    KEY_DOWN,
    KEY_RED,
    KEY_GREEN,
    KEY_BLUE,
    KEY_YELLOW,
    KEY_0,
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_6,
    KEY_7,
    KEY_8,
    KEY_9,
};

uint16_t tick_count;

uint8_t tick_handle_event(uint8_t e)
{
    if (e != EVENT_10MS_TICK)
        return 0;
    tick_count++;
    return 1;
}

uint16_t last_code_tick = 0;
uint8_t last_code = 0xFF;
uint8_t repeate_code = 0;

void rc_key_pressed(uint8_t code)
{
    if (code != last_code)
    {
        // Different key, just store code and time, mark that it's not
        // repetition and check key later
        last_code = code;
        last_code_tick = tick_count;
        repeate_code = 0;
    }
    else
    {
        uint16_t diff;
        uint16_t t;
        __disable_interrupt();
        t = tick_count;
        __enable_interrupt();
        diff = tick_count - last_code_tick;
        printf("%d\n", diff);
        // Last press was very short time ago do nothing
        if (diff < 10)
            return;
        // Last press was slightly later but it's first time after
        // press wait a little bit longer if this is first time
        if (repeate_code == 0 && diff < 25)
            return;
        // If previous press time was some time ago treat it as first press
        repeate_code = diff < 50;
        last_code_tick = t;
    }

    switch (code)
    {
    case KEY_POWER:
        power();
        break;
    case KEY_UP:
        up();
        break;
    case KEY_DOWN:
        down();
        break;
    case KEY_RED:
        red();
        break;
    case KEY_GREEN:
        green();
        break;
    case KEY_BLUE:
        blue();
        break;
    case KEY_YELLOW:
        yellow();
        break;
    case KEY_0:
    case KEY_1:
    case KEY_2:
    case KEY_3:
    case KEY_4:
    case KEY_5:
    case KEY_6:
    case KEY_7:
    case KEY_8:
    case KEY_9:
        digit(code - KEY_0);
        break;
    }
}

const uint8_t rc_codes[] =
{
    RC_CODE1( 0x4C, KEY_UP ),
    RC_CODE1( 0x13, KEY_UP ),
    RC_CODE1( 0x4D, KEY_DOWN ),
    RC_CODE1( 0x14, KEY_DOWN ),
    RC_CODE1( 0x32, KEY_RED ),
    RC_CODE1( 0x33, KEY_GREEN ),
    RC_CODE1( 0x35, KEY_BLUE ),
    RC_CODE1( 0x34, KEY_YELLOW ),
    RC_CODE1( 0x15, KEY_POWER ),
    RC_CODE1( 0x01, KEY_1),
    RC_CODE1( 0x02, KEY_2),
    RC_CODE1( 0x03, KEY_3),
    RC_CODE1( 0x04, KEY_4),
    RC_CODE1( 0x05, KEY_5),
    RC_CODE1( 0x06, KEY_6),
    RC_CODE1( 0x07, KEY_7),
    RC_CODE1( 0x08, KEY_8),
    RC_CODE1( 0x09, KEY_9),
    RC_CODE1( 0x00, KEY_0),
    RC_CODE4( 0x03, 0xFC, 0x04, 0xFB, KEY_UP),
    RC_CODE4( 0x03, 0xFC, 0x08, 0xF7, KEY_DOWN),
    RC_CODE4( 0x03, 0xFC, 0x00, 0xFF, KEY_POWER),
    RC_CODE4( 0x03, 0xFC, 0x4B, 0xB4, KEY_RED),
    RC_CODE4( 0x03, 0xFC, 0x4C, 0xB3, KEY_GREEN),
    RC_CODE4( 0x03, 0xFC, 0x4E, 0xB1, KEY_BLUE),
    RC_CODE4( 0x03, 0xFC, 0x4D, 0xB2, KEY_YELLOW),
    RC_CODE4( 0x03, 0xFC, 0x05, 0xFA, KEY_1),
    RC_CODE4( 0x03, 0xFC, 0x06, 0xF9, KEY_2),
    RC_CODE4( 0x03, 0xFC, 0x07, 0xF8, KEY_3),
    RC_CODE4( 0x03, 0xFC, 0x09, 0xF6, KEY_4),
    RC_CODE4( 0x03, 0xFC, 0x0A, 0xF5, KEY_5),
    RC_CODE4( 0x03, 0xFC, 0x0B, 0xF4, KEY_6),
    RC_CODE4( 0x03, 0xFC, 0x0D, 0xF2, KEY_7),
    RC_CODE4( 0x03, 0xFC, 0x0E, 0xF1, KEY_8),
    RC_CODE4( 0x03, 0xFC, 0x0F, 0xF0, KEY_9),
    RC_CODE4( 0x03, 0xFC, 0x12, 0xED, KEY_0),

    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x34, 0xB4, KEY_UP),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x35, 0xB5, KEY_DOWN),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x3D, 0xBD, KEY_POWER),
    RC_CODE6( 0x02, 0x20, 0xB0, 0x00, 0x3D, 0x8D, KEY_POWER),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x70, 0xF0, KEY_RED),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x71, 0xF1, KEY_GREEN),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x73, 0xF3, KEY_BLUE),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x72, 0xF2, KEY_YELLOW),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x10, 0x90, KEY_1),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x11, 0x91, KEY_2),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x12, 0x92, KEY_3),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x13, 0x93, KEY_4),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x14, 0x94, KEY_5),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x15, 0x95, KEY_6),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x16, 0x96, KEY_7),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x17, 0x97, KEY_8),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x18, 0x98, KEY_9),
    RC_CODE6( 0x02, 0x20, 0x80, 0x00, 0x19, 0x99, KEY_0),
    0,
};

struct event_handler event_handlers[] =
{
    { rc_handle_event },
    { delayed_write_handle_event },
    { tick_handle_event },
    { awu_handle_event },
    { NULL }
};

void __noreturn main(void)
{
    // HSI 16 MHz
    //   HSIDIV = 1
    CLK_CKDIVR_HSIDIV = 0;

    restore_eeprom_values();

#ifndef NDEBUG
    set_baudrate(BAURATE_115200);
    UART1_CR2_TEN = 1;
#endif

    PD_DDR_DDR2 = 1; // Output
    PD_DDR_DDR3 = 1;
    PD_ODR_ODR2 = 0; // low
    PD_ODR_ODR3 = 0;
    PD_CR1_C12 = 1; // Push pull
    PD_CR1_C13 = 1;
    PD_CR2_C22 = 1; // 10MHz
    PD_CR2_C23 = 1;

    update_leds();

    init_tick();
    init_rc();
    __enable_interrupt();
    event_loop();
}
