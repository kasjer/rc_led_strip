#ifndef _RC_H
#define _RC_H

#ifndef RC_CODE_LEN
#define RC_CODE_LEN 6
#endif

#define EVENT_BIT_END           1
#define EVENT_HI_END            2
#define EVENT_DATA_END          4

uint8_t rc_handle_event(uint8_t e);
void init_rc(void);
// User must define this function
void rc_key_pressed(uint8_t code);
// User may define this function
void rc_default(uint8_t len, const uint8_t *data);

#define RC_CODE1(a, code) 1, code, a
#define RC_CODE4(a, b, c, d, code) 4, code, a, b, c, d
#define RC_CODE6(a, b, c, d, e, f, code) 6, code, a, b, c, d, e, f

struct rc_code
{
    uint8_t len;
    uint8_t key_code;
    uint8_t code[];
};

extern const uint8_t rc_codes[];

#endif /* _RC_H */
