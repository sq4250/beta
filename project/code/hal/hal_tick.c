#include "zf_common_headfile.h"
#include "hal_tick.h"

static volatile u32 s_tick;
static void (*s_handler)(void);
static bool s_ok;

void hal_tick_callback(void) { s_tick++; if (s_handler) s_handler(); }

void hal_tick_init(void (*handler)(void)) {
    if (s_ok) return;
    s_ok = true; s_handler = handler;
    pit_ms_init(PIT_CH1, 1);
}

u32 hal_tick_get(void) { return s_tick; }
