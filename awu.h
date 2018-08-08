#ifndef _AWU_H
#define _AWU_H

#define EVENT_AWU   10

uint8_t awu_handle_event(uint8_t e);
void set_awu_timeout(uint8_t seconds, uint8_t event);

#endif
