#ifndef _DISPATCHER_H
#define _DISPATCHER_H

#include <stdint.h>

typedef uint8_t event_t;

struct event_handler
{
    uint8_t (*fn)(event_t e);
};

extern uint8_t events[8];
extern uint8_t event_count;
extern uint8_t event_ptr;

extern struct event_handler event_handlers[];

void __noreturn event_loop(void);

#define ADD_EVENT(e) do { events[event_ptr & 7] = e; event_ptr++; event_count++; } while (0)

#endif /* _DISPATCHER_H */
