/* ESP32-2432S028R "Cheap Yellow Display" (ESP32, 2.8" 240x320 ILI9341 or
 * ST7789, XPT2046 resistive touch) stand-alone spectrum and waterfall. Runs while no serial
 * host holds the lease. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* One snapshot of n IQ10 words at a ring rate code, with the given analog
 * bandwidth; failures are not reported to the host. */
typedef bool (*cyd_acquire_fn)(unsigned n, unsigned rate_code, unsigned bandwidth_mhz, const uint32_t **words,
                               unsigned *elapsed_us);
typedef void (*cyd_retune_fn)(unsigned khz);

bool cyd_init(void);
void cyd_step(unsigned center_khz, bool host_active, cyd_acquire_fn acquire, cyd_retune_fn retune);
/* CYD?, CYDLINE, CYDCFG, CYDTEST: ordinary commands. */
bool cyd_command(const char *line);
/* CYDTOUCH?: passive (no lease, display keeps running). */
bool cyd_passive_command(const char *line);
