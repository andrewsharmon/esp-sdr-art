/* ESP32-2432S028R "Cheap Yellow Display" live spectrum + waterfall.
 *
 * The original ESP32 only has snapshot capture. Peak hold transforms the
 * strongest nfft-sample blocks of each snapshot (plus one rotating block
 * for the floor), so short bursts are caught far more often than with
 * consecutive blocks; average transforms every block. Lines are drawn
 * about ten times a second, in portrait (240x320, USB facing down) or
 * landscape. Only commands that ILI9341 and ST7789 panels share are sent,
 * so both board revisions work; orientation and colour order come from
 * MADCTL (CYDCFG).
 *
 * Touch: left / right third = step down / up (frequency in TUNE, span in
 * SPAN); middle = switch TUNE <-> SPAN; long press in the middle = peak
 * hold <-> average. The BOOT button mirrors the middle. */
#include "cyd_ui.h"
#include "burst_serial.h"
#include "spectrum.h"
#include "spectrum_dc.h"
#include "spectrum_fft.h"
#include "spectrum_math.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "dsps_fft2r.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PIN_LCD_MOSI 13
#define PIN_LCD_SCLK 14
#define PIN_LCD_CS   15
#define PIN_LCD_DC   2
#define PIN_LCD_BL   21
#define PIN_TP_CLK   25
#define PIN_TP_MOSI  32
#define PIN_TP_MISO  39
#define PIN_TP_CS    33
#define PIN_TP_IRQ   36
#define PIN_BOOT     0
#define PIN_LED_R    4
#define PIN_LED_G    16
#define PIN_LED_B    17
#define PIN_SPEAKER  26

#if CONFIG_ESP_SDR_CYD_PORTRAIT
#define W        240 /* USB ports facing down */
#define H        320
#define SPEC_END 116
#else
#define W        320
#define H        240
#define SPEC_END 96
#endif
#define STRIP          10
#define HEADER_END     22
#define AXIS_END       36
#define SPEC_ROWS      (SPEC_END - AXIS_END)
#define WF_ROWS        (H - SPEC_END)
#define MAX_NFFT       2048u
#define SNAPSHOT_WORDS 16380u
#define STRONGEST      3u
#define LINE_US        100000
#define LONG_PRESS_US  700000
#define REPEAT_US      400000
#define REPEAT_EVERY   120000
#define FREQ_MIN_KHZ   2210000u /* keep clear of the 1842-2210 MHz alternate LO plan */
#define FREQ_MAX_KHZ   2700000u
#define LCD_HZ         20000000

typedef struct {
    uint16_t span_mhz_x10;
    uint8_t rate; /* 0 = 80, 1 = 40, 6 = 16 MS/s */
    uint16_t nfft;
    uint8_t bandwidth_mhz;
    uint16_t step_khz;
} span_t;
static const span_t spans[] = {
    {800, 0, 1024, 67, 5000}, {600, 0, 1024, 62, 5000}, {400, 1, 1024, 40, 2000}, {200, 1, 2048, 24, 1000},
    {160, 6, 1024, 14, 1000}, {100, 6, 2048, 12, 500},  {50, 6, 2048, 12, 250},   {20, 6, 2048, 12, 100},
};
#define SPAN_COUNT (sizeof(spans) / sizeof(spans[0]))
static unsigned rate_hz(unsigned code) {
    return code == 0 ? 80000000u : code == 1 ? 40000000u : 16000000u;
}

/* ---- State -------------------------------------------------------------- */

static esp_lcd_panel_io_handle_t io;
static SemaphoreHandle_t strip_done;
static uint16_t *strip[2];
static spi_device_handle_t touch;
static bool present;
static const char *init_failure = "init";
/* Panel settings vary across CYD revisions (ILI9341 or ST7789, mounting).
 * CYDCFG / CYDTEST store overrides in NVS so they survive the auto-reset
 * that opening the serial port triggers. */
static uint8_t cfg_madctl, cfg_invert, cfg_test;

static unsigned span_index;
static bool knob_span; /* false: TUNE, true: SPAN */
static bool average;

static int16_t *fft, *window; /* spectrum.c workspace, idle while no host */
static uint32_t *peak;        /* per FFT slot: uint32 power, or float sum when averaging */
static unsigned window_n, line_ffts, rotating_block, lines;
static int64_t line_start;
static spectrum_dc_t dc;

static uint8_t column[W];
static uint8_t *waterfall; /* WF_ROWS x W palette levels, ring */
static unsigned wf_head;
static int floor_q4 = -1; /* 0.5 dB codes << 4 */
static int peak_offset_hz, peak_db;
static uint16_t palette[256]; /* byte-swapped RGB565 */

static int touch_x = -1, touch_raw_x, touch_raw_y, touch_raw_z;
static int64_t press_start, last_repeat;
static int press_zone; /* -1 none, 0 left, 1 middle, 2 right, 3 BOOT */
static bool long_fired;

/* ---- Colours and font ---------------------------------------------------- */

static uint16_t rgb(unsigned r, unsigned g, unsigned b) {
    uint16_t c = (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8)); /* panel takes big-endian pixels */
}

static void build_palette(void) {
    static const uint8_t stops[][4] = {{0, 0, 0, 0},       {40, 0, 0, 110},     {90, 0, 90, 220},
                                       {130, 0, 210, 210}, {170, 40, 220, 40},  {205, 250, 230, 0},
                                       {235, 255, 60, 0},  {255, 255, 255, 255}};
    for (unsigned s = 0; s + 1 < sizeof(stops) / sizeof(stops[0]); s++)
        for (unsigned v = stops[s][0]; v <= stops[s + 1][0]; v++) {
            unsigned span = stops[s + 1][0] - stops[s][0], t = v - stops[s][0];
            unsigned c[3];
            for (unsigned k = 0; k < 3; k++)
                c[k] = (stops[s][k + 1] * (span - t) + stops[s + 1][k + 1] * t) / span;
            palette[v] = rgb(c[0], c[1], c[2]);
        }
}

/* Classic 5x7, columns LSB-top. */
static const uint8_t *glyph(char c) {
    static const uint8_t digits[10][5] = {{0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00},
                                          {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31},
                                          {0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
                                          {0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
                                          {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1e}};
    static const uint8_t letters[26][5] = {
        {0x7e, 0x11, 0x11, 0x11, 0x7e}, {0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22},
        {0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41}, {0x7f, 0x09, 0x09, 0x09, 0x01},
        {0x3e, 0x41, 0x49, 0x49, 0x7a}, {0x7f, 0x08, 0x08, 0x08, 0x7f}, {0x00, 0x41, 0x7f, 0x41, 0x00},
        {0x20, 0x40, 0x41, 0x3f, 0x01}, {0x7f, 0x08, 0x14, 0x22, 0x41}, {0x7f, 0x40, 0x40, 0x40, 0x40},
        {0x7f, 0x02, 0x0c, 0x02, 0x7f}, {0x7f, 0x04, 0x08, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e},
        {0x7f, 0x09, 0x09, 0x09, 0x06}, {0x3e, 0x41, 0x51, 0x21, 0x5e}, {0x7f, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7f, 0x01, 0x01}, {0x3f, 0x40, 0x40, 0x40, 0x3f},
        {0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x3f, 0x40, 0x38, 0x40, 0x3f}, {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00}, colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00},
                         minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08}, plus[5] = {0x08, 0x08, 0x3e, 0x08, 0x08},
                         slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02}, less[5] = {0x08, 0x14, 0x22, 0x41, 0x00},
                         greater[5] = {0x00, 0x41, 0x22, 0x14, 0x08}, blank[5] = {0};
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
    switch (c) {
    case '.': return dot;
    case ':': return colon;
    case '-': return minus;
    case '+': return plus;
    case '/': return slash;
    case '<': return less;
    case '>': return greater;
    default: return blank;
    }
}

/* Draw into the strip covering rows [y0, y0 + STRIP). */
static void text(uint16_t *buf, int y0, int x, int y, int scale, uint16_t color, const char *s) {
    for (; *s; s++, x += 6 * scale) {
        const uint8_t *g = glyph(*s);
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 7; row++) {
                if (!(g[col] & (1u << row))) continue;
                for (int dy = 0; dy < scale; dy++) {
                    int py = y + row * scale + dy - y0;
                    if (py < 0 || py >= STRIP) continue;
                    for (int dx = 0; dx < scale; dx++) {
                        int px = x + col * scale + dx;
                        if (px >= 0 && px < W) buf[py * W + px] = color;
                    }
                }
            }
    }
}

static int text_width(const char *s, int scale) {
    return (int)strlen(s) * 6 * scale - scale;
}

static void fill(uint16_t *buf, int y0, int x, int y, int w, int h, uint16_t color) {
    for (int py = y; py < y + h; py++) {
        if (py < y0 || py >= y0 + STRIP) continue;
        for (int px = x; px < x + w; px++)
            if (px >= 0 && px < W) buf[(py - y0) * W + px] = color;
    }
}

/* ---- ILI9341 ------------------------------------------------------------- */

static bool on_strip_done(esp_lcd_panel_io_handle_t handle, esp_lcd_panel_io_event_data_t *event, void *ctx) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(strip_done, &woken);
    return woken == pdTRUE;
}

static esp_err_t io_open(unsigned hz) {
    if (io) esp_lcd_panel_io_del(io);
    io = NULL;
    esp_lcd_panel_io_spi_config_t config = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = hz,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_strip_done,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    return esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &config, &io);
}

static void cmd(uint8_t command, const void *data, size_t size) {
    esp_lcd_panel_io_tx_param(io, command, data, size);
}

static void config_load(void) {
    cfg_madctl = CONFIG_ESP_SDR_CYD_MADCTL;
#if CONFIG_ESP_SDR_CYD_INVERT
    cfg_invert = 1;
#else
    cfg_invert = 0;
#endif
    cfg_test = 0;
    nvs_handle_t nvs;
    if (nvs_open("cyd", NVS_READONLY, &nvs) != ESP_OK) return;
    nvs_get_u8(nvs, "madctl", &cfg_madctl);
    nvs_get_u8(nvs, "invert", &cfg_invert);
    nvs_get_u8(nvs, "test", &cfg_test);
    nvs_close(nvs);
}

static void config_save(void) {
    nvs_handle_t nvs;
    if (nvs_open("cyd", NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u8(nvs, "madctl", cfg_madctl);
    nvs_set_u8(nvs, "invert", cfg_invert);
    nvs_set_u8(nvs, "test", cfg_test);
    nvs_commit(nvs);
    nvs_close(nvs);
}

static void set_window(int x0, int y0, int x1, int y1) {
    uint8_t c[4] = {x0 >> 8, x0 & 255, x1 >> 8, x1 & 255}, r[4] = {y0 >> 8, y0 & 255, y1 >> 8, y1 & 255};
    cmd(0x2a, c, 4);
    cmd(0x2b, r, 4);
}

/* Commands both ILI9341 and ST7789 accept with the same meaning; vendor
 * power/gamma tables differ between the two and garble the other. */
static void panel_setup(void) {
    cmd(0x01, NULL, 0); /* software reset: the panel's RST is tied to EN */
    vTaskDelay(pdMS_TO_TICKS(150));
    cmd(0x11, NULL, 0); /* sleep out */
    vTaskDelay(pdMS_TO_TICKS(120));
    uint8_t colmod = 0x55;
    cmd(0x3a, &colmod, 1);
    cmd(0x36, &cfg_madctl, 1);
    cmd(cfg_invert ? 0x21 : 0x20, NULL, 0);
    cmd(0x13, NULL, 0); /* normal display mode */
    cmd(0x29, NULL, 0); /* display on */
    vTaskDelay(pdMS_TO_TICKS(20));
}

static bool lcd_init(void) {
    gpio_set_direction(PIN_LCD_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LCD_BL, 0);
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1, /* the two-USB board does not wire the panel's data out */
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = W * STRIP * 2 + 16,
    };
    init_failure = "spi";
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;
    strip_done = xSemaphoreCreateCounting(2, 2);
    init_failure = "lcd_io";
    if (io_open(LCD_HZ) != ESP_OK) return false;
    panel_setup();
    init_failure = "memory";
    for (int k = 0; k < 2; k++)
        if (!(strip[k] = heap_caps_malloc(W * STRIP * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL))) return false;
    return true;
}

/* ---- XPT2046 touch -------------------------------------------------------- */

static int touch_read(uint8_t command);

static bool touch_init(void) {
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_TP_MOSI,
        .miso_io_num = PIN_TP_MISO,
        .sclk_io_num = PIN_TP_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    init_failure = "touch";
    if (spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK) return false;
    spi_device_interface_config_t device = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_TP_CS,
        .queue_size = 1,
    };
    if (spi_bus_add_device(SPI3_HOST, &device, &touch) != ESP_OK) return false;
    gpio_set_direction(PIN_TP_IRQ, GPIO_MODE_INPUT);
    touch_read(0xd0); /* power-down mode 00: pen interrupt enabled */
    return true;
}

static int touch_read(uint8_t command) {
    uint8_t tx[3] = {command, 0, 0}, rx[3] = {0};
    spi_transaction_t t = {.length = 24, .tx_buffer = tx, .rx_buffer = rx};
    if (spi_device_polling_transmit(touch, &t) != ESP_OK) return -1;
    return ((rx[1] << 8) | rx[2]) >> 3;
}

/* Screen x of the current touch, or -1. Raw axes are 12-bit. Every
 * conversion uses power-down mode 00, which keeps the pen interrupt enabled:
 * a read that left the controller in mode 01 (reference on, PENIRQ off)
 * would silence the IRQ line, and with it every later touch. */
static int touch_screen_x(void) {
    if (gpio_get_level(PIN_TP_IRQ)) return -1;
    int z1 = touch_read(0xb0), z2 = touch_read(0xc0);
    touch_raw_z = z1 + 4095 - z2;
    if (touch_raw_z < 400) return -1;
    int x = 0, y = 0;
    touch_read(0xd0); /* settle */
    for (int k = 0; k < 4; k++) {
        x += touch_read(0xd0);
        y += touch_read(0x90);
    }
    touch_raw_x = x / 4;
    touch_raw_y = y / 4;
#if CONFIG_ESP_SDR_CYD_TOUCH_SWAP_XY
    int raw = touch_raw_y;
#else
    int raw = touch_raw_x;
#endif
    int sx = (raw - 200) * W / 3500;
#if CONFIG_ESP_SDR_CYD_TOUCH_INVERT_X
    sx = W - 1 - sx;
#endif
    return sx < 0 ? 0 : sx >= W ? W - 1 : sx;
}

/* ---- Rendering ------------------------------------------------------------ */

static void format_mhz(char *out, size_t size, long khz, int decimals) {
    long whole = khz / 1000, frac = khz % 1000;
    if (frac < 0) frac = -frac;
    if (decimals == 3)
        snprintf(out, size, "%ld.%03ld", whole, frac);
    else if (decimals == 2)
        snprintf(out, size, "%ld.%02ld", whole, frac / 10);
    else
        snprintf(out, size, "%ld.%ld", whole, frac / 100);
}

static void format_span(char *out, size_t size, unsigned x10) {
    if (x10 % 10)
        snprintf(out, size, "%u.%uM", x10 / 10, x10 % 10);
    else
        snprintf(out, size, "%uM", x10 / 10);
}

static void render_strip(uint16_t *buf, int y0, unsigned center_khz, bool host_active) {
    const uint16_t header_bg = rgb(18, 24, 40), black = 0, white = rgb(255, 255, 255),
                   grey = rgb(110, 120, 140), grid = rgb(40, 48, 64), accent = rgb(255, 176, 0),
                   cyan = rgb(80, 220, 255), trace = rgb(255, 255, 160);
    const int floor_code = floor_q4 < 0 ? 0 : floor_q4 >> 4;
    for (int row = 0; row < STRIP; row++) {
        int y = y0 + row;
        uint16_t *line = buf + row * W;
        if (y >= H) break;
        if (y < HEADER_END) {
            for (int x = 0; x < W; x++) line[x] = header_bg;
        } else if (y < AXIS_END) {
            for (int x = 0; x < W; x++) line[x] = black;
        } else if (y < SPEC_END) {
            int level = SPEC_END - 1 - y;                          /* 0 at the bottom, 50 dB over SPEC_ROWS */
            bool grid_row = level && level % (SPEC_ROWS / 5) == 0; /* every 10 dB */
            for (int x = 0; x < W; x++) {
                int height = ((int)column[x] - floor_code + 6) * SPEC_ROWS / 100;
                if (level == height - 1 || (level == 0 && height <= 0))
                    line[x] = trace;
                else if (level < height)
                    line[x] = palette[64 + level * 180 / SPEC_ROWS];
                else
                    line[x] = grid_row && !(x & 3) ? grid : black;
            }
        } else {
            unsigned age = (unsigned)(y - SPEC_END);
            const uint8_t *src = waterfall + ((wf_head + WF_ROWS - age) % WF_ROWS) * W;
            for (int x = 0; x < W; x++) line[x] = palette[src[x]];
        }
    }

    char a[48], b[32];
    format_mhz(a, sizeof(a), center_khz, 3);
    int fw = text_width(a, 2);
    if (!knob_span) fill(buf, y0, 0, 0, fw + 8, HEADER_END, accent);
    text(buf, y0, 4, 4, 2, knob_span ? white : black, a);
    text(buf, y0, fw + 11, 12, 1, grey, "MHZ");
    text(buf, y0, fw + 11, 3, 1, average ? cyan : accent, average ? "AVG" : "PK");
    format_span(b, sizeof(b), spans[span_index].span_mhz_x10);
    snprintf(a, sizeof(a), "SPAN %s", b);
    int sw = text_width(a, 2);
    if (knob_span) fill(buf, y0, W - sw - 8, 0, sw + 8, HEADER_END, accent);
    text(buf, y0, W - sw - 4, 4, 2, knob_span ? black : white, a);

    long half = spans[span_index].span_mhz_x10 * 50L;
    int decimals = spans[span_index].span_mhz_x10 >= 100 ? 1 : 2;
    if (host_active) {
        text(buf, y0, (W - text_width("USB HOST - DISPLAY PAUSED", 1)) / 2, 25, 1, accent,
             "USB HOST - DISPLAY PAUSED");
    } else {
        format_mhz(a, sizeof(a), (long)center_khz - half, decimals);
        text(buf, y0, 0, 25, 1, grey, a);
        format_mhz(a, sizeof(a), (long)center_khz + half, decimals);
        text(buf, y0, W - text_width(a, 1), 25, 1, grey, a);
        if (lines) {
            format_mhz(b, sizeof(b), (long)center_khz + peak_offset_hz / 1000, 2);
            snprintf(a, sizeof(a), "PK %s +%dDB", b, peak_db);
            text(buf, y0, (W - text_width(a, 1)) / 2, 25, 1, cyan, a);
        }
    }
    for (int k = 0; k <= 10; k++) {
        int x = k * (W - 1) / 10;
        fill(buf, y0, x, k % 5 ? 34 : 33, 1, k % 5 ? 2 : 3, k == 5 ? accent : grey);
    }
    /* Touch zones: step down / up at the edges. */
    text(buf, y0, 4, AXIS_END + 4, 2, grey, "<");
    text(buf, y0, W - 14, AXIS_END + 4, 2, grey, ">");
}

/* Orientation and colour check: labelled corners, TOP marker, colour bars. */
static void render_test_strip(uint16_t *buf, int y0) {
    static const uint8_t bars[8][3] = {{255, 255, 255}, {255, 255, 0}, {0, 255, 255}, {0, 255, 0},
                                       {255, 0, 255},   {255, 0, 0},   {0, 0, 255},   {0, 0, 0}};
    for (int row = 0; row < STRIP; row++) {
        int y = y0 + row;
        for (int x = 0; x < W; x++) {
            const uint8_t *c = bars[x * 8 / W];
            buf[row * W + x] = (y >= 80 && y < 160) ? rgb(c[0], c[1], c[2]) : rgb(30, 30, 30);
        }
    }
    fill(buf, y0, 0, 0, 48, 48, rgb(255, 0, 0));
    fill(buf, y0, W - 48, 0, 48, 48, rgb(0, 255, 0));
    fill(buf, y0, 0, H - 48, 48, 48, rgb(0, 0, 255));
    fill(buf, y0, W - 48, H - 48, 48, 48, rgb(255, 255, 255));
    text(buf, y0, 12, 17, 2, rgb(255, 255, 255), "TL");
    text(buf, y0, W - 36, 17, 2, rgb(0, 0, 0), "TR");
    text(buf, y0, 12, H - 31, 2, rgb(255, 255, 255), "BL");
    text(buf, y0, W - 36, H - 31, 2, rgb(0, 0, 0), "BR");
    text(buf, y0, (W - text_width("TOP", 3)) / 2, 10, 3, rgb(255, 255, 255), "TOP");
    text(buf, y0, (W - text_width("CYD TEST", 2)) / 2, 180, 2, rgb(255, 255, 255), "CYD TEST");
}

static void render(unsigned center_khz, bool host_active) {
    set_window(0, 0, W - 1, H - 1);
    for (int y0 = 0, k = 0; y0 < H; y0 += STRIP, k ^= 1) {
        xSemaphoreTake(strip_done, portMAX_DELAY);
        if (cfg_test)
            render_test_strip(strip[k], y0);
        else
            render_strip(strip[k], y0, center_khz, host_active);
        esp_lcd_panel_io_tx_color(io, y0 ? 0x3c : 0x2c, strip[k], W * STRIP * 2);
    }
    xSemaphoreTake(strip_done, portMAX_DELAY);
    xSemaphoreTake(strip_done, portMAX_DELAY);
    xSemaphoreGive(strip_done);
    xSemaphoreGive(strip_done);
}

/* ---- Signal processing ---------------------------------------------------- */

static void no_poll(void *context) {
    (void)context;
}

static unsigned log2u(unsigned n) {
    unsigned l = 0;
    while ((1u << l) < n) l++;
    return l;
}

static unsigned bit_reverse(unsigned value, unsigned bits) {
    unsigned result = 0;
    for (unsigned j = 0; j < bits; j++, value >>= 1) result = (result << 1) | (value & 1);
    return result;
}

static void workspace(unsigned n) {
    spectrum_workspace_t ws = spectrum_workspace();
    fft = ws.fft;
    window = ws.window;
    peak = (uint32_t *)ws.power;
    if (window_n != n) {
        for (unsigned j = 0; j < n; j++)
            window[j] = (int16_t)lrintf(16383.5f * (1 - cosf(2 * (float)M_PI * j / n)));
        window_n = n;
        memset(peak, 0, MAX_NFFT * sizeof(uint32_t));
        line_ffts = 0;
    }
}

static void transform(const uint32_t *words, unsigned n) {
    for (unsigned j = 0; j < n; j++) {
        uint32_t w = words[j];
        int32_t i = (int32_t)(w << 22) >> 22, q = (int32_t)(w << 12) >> 22;
        fft[2 * j] = spectrum_sat16((i * window[j]) >> 9);
        fft[2 * j + 1] = spectrum_sat16((q * window[j]) >> 9);
    }
    spectrum_fft_poll(fft, n, dsps_fft_w_table_sc16, no_poll, NULL);
    spectrum_dc_apply(&dc, fft, n);
    float *sum = (float *)peak;
    for (unsigned k = 0; k < n; k++) {
        uint32_t p = spectrum_complex_power(fft[2 * k], fft[2 * k + 1]);
        if (average)
            sum[k] += (float)p;
        else if (p > peak[k])
            peak[k] = p;
    }
    line_ffts++;
}

static void process_snapshot(const uint32_t *words, unsigned n, unsigned blocks) {
    if (average) {
        for (unsigned b = 0; b < blocks; b++) transform(words + b * n, n);
        return;
    }
    uint32_t energy[SNAPSHOT_WORDS / 1024] = {0};
    unsigned chosen[STRONGEST + 1], picks = 0;
    for (unsigned b = 0; b < blocks; b++) {
        uint32_t e = 0;
        for (unsigned j = b * n; j < (b + 1) * n; j += 4) { /* every 4th sample is plenty to rank */
            int32_t i = (int32_t)(words[j] << 22) >> 22, q = (int32_t)(words[j] << 12) >> 22;
            e += (uint32_t)(i * i + q * q);
        }
        energy[b] = e;
    }
    for (; picks < STRONGEST && picks < blocks; picks++) {
        unsigned best = 0;
        for (unsigned b = 1; b < blocks; b++)
            if (energy[b] > energy[best]) best = b;
        chosen[picks] = best;
        energy[best] = 0;
    }
    rotating_block = (rotating_block + 1) % blocks;
    if (energy[rotating_block]) chosen[picks++] = rotating_block;
    for (unsigned k = 0; k < picks; k++) transform(words + chosen[k] * n, n);
}

static uint8_t bin_code(unsigned n, unsigned log2n, unsigned d) {
    unsigned slot = bit_reverse((n / 2 - d) & (n - 1), log2n); /* conj(I+jQ): display bin d */
    if (average) return spectrum_mean_power_code(((float *)peak)[slot] / (float)line_ffts);
    return spectrum_power_code(peak[slot]);
}

static void finish_line(void) {
    const span_t *s = &spans[span_index];
    unsigned n = s->nfft, log2n = log2u(n), rate = rate_hz(s->rate);
    unsigned shown = (unsigned)((uint64_t)n * s->span_mhz_x10 * 100000u / rate);
    unsigned first = n / 2 - shown / 2, dc_bin = n / 2;
    unsigned histogram[256] = {0}, best = 0, best_d = dc_bin;
    for (unsigned x = 0; x < W; x++) {
        unsigned d0 = first + x * shown / W, d1 = first + (x + 1) * shown / W;
        if (d1 <= d0) d1 = d0 + 1;
        unsigned value = 0;
        bool any = false;
        for (unsigned d = d0; d < d1; d++) {
            if (d + 1 >= dc_bin && d <= dc_bin + 1) continue; /* zero-IF DC / LO leakage */
            unsigned v = bin_code(n, log2n, d);
            if (v > value) value = v;
            any = true;
            if (v > best) {
                best = v;
                best_d = d;
            }
        }
        if (!any) {
            unsigned l = bin_code(n, log2n, dc_bin - 2), r = bin_code(n, log2n, dc_bin + 2);
            value = l > r ? l : r;
        }
        column[x] = value;
        histogram[value]++;
    }
    memset(peak, 0, n * sizeof(uint32_t));
    line_ffts = 0;

    unsigned count = 0, percentile = 0;
    while (percentile < 255 && (count += histogram[percentile]) < W / 5) percentile++;
    if (floor_q4 < 0)
        floor_q4 = percentile << 4;
    else
        floor_q4 += ((int)(percentile << 4) - floor_q4) / 3;
    int floor_code = floor_q4 >> 4;
    peak_offset_hz = (int)(((int64_t)best_d - (int64_t)dc_bin) * rate / n);
    peak_db = ((int)best - floor_code) / 2;
    if (peak_db < 0) peak_db = 0;

    wf_head = (wf_head + 1) % WF_ROWS;
    uint8_t *row = waterfall + wf_head * W;
    for (unsigned x = 0; x < W; x++) {
        int level = ((int)column[x] - floor_code - 4) * 255 / 80; /* 40 dB */
        row[x] = level < 0 ? 0 : level > 255 ? 255 : (uint8_t)level;
    }
    lines++;
}

/* ---- Controls --------------------------------------------------------------- */

static void reset_display_history(void) {
    memset(waterfall, 0, WF_ROWS * W);
    memset(column, 0, sizeof(column));
    floor_q4 = -1;
    if (peak) memset(peak, 0, MAX_NFFT * sizeof(uint32_t));
    line_ffts = 0;
}

static void step(int direction, unsigned center_khz, cyd_retune_fn retune) {
    if (knob_span) {
        int next = (int)span_index + direction; /* right zooms in */
        next = next < 0 ? 0 : next >= (int)SPAN_COUNT ? (int)SPAN_COUNT - 1 : next;
        if ((unsigned)next != span_index) {
            span_index = next;
            reset_display_history();
        }
    } else {
        int s = spans[span_index].step_khz;
        int64_t khz = (int64_t)center_khz / s * s + (int64_t)direction * s;
        if (khz < FREQ_MIN_KHZ) khz = FREQ_MIN_KHZ;
        if (khz > FREQ_MAX_KHZ) khz = FREQ_MAX_KHZ;
        retune((unsigned)khz);
    }
}

static void poll_controls(unsigned center_khz, cyd_retune_fn retune) {
    int64_t now = esp_timer_get_time();
    touch_x = touch_screen_x();
    int zone = touch_x >= 0 ? (touch_x < W / 3 ? 0 : touch_x < 2 * W / 3 ? 1 : 2) : -1;
    if (zone < 0 && !gpio_get_level(PIN_BOOT)) zone = 3;
    if (zone >= 0 && press_zone < 0) {
        press_zone = zone;
        press_start = last_repeat = now;
        long_fired = false;
        if (zone == 0 || zone == 2) step(zone == 0 ? -1 : 1, center_khz, retune);
    } else if (zone >= 0 && zone == press_zone) {
        if ((zone == 0 || zone == 2) && now - press_start >= REPEAT_US && now - last_repeat >= REPEAT_EVERY) {
            last_repeat = now;
            step(zone == 0 ? -1 : 1, center_khz, retune);
        } else if ((zone == 1 || zone == 3) && !long_fired && now - press_start >= LONG_PRESS_US) {
            long_fired = true;
            average = !average;
            reset_display_history();
        }
    } else if (zone < 0 && press_zone >= 0) {
        if ((press_zone == 1 || press_zone == 3) && !long_fired && now - press_start > 30000)
            knob_span = !knob_span;
        press_zone = -1;
    }
}

/* ---- Public ------------------------------------------------------------------ */

bool cyd_init(void) {
    /* RGB LED (active low) off, speaker quiet, BOOT as a button. */
    const int leds[] = {PIN_LED_R, PIN_LED_G, PIN_LED_B};
    for (unsigned k = 0; k < 3; k++) {
        gpio_set_direction(leds[k], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[k], 1);
    }
    gpio_set_direction(PIN_SPEAKER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SPEAKER, 0);
    gpio_config_t boot = {
        .pin_bit_mask = 1ull << PIN_BOOT, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&boot);
    build_palette();
    config_load();
    press_zone = -1;
    init_failure = "memory";
    if (!(waterfall =
              heap_caps_calloc(WF_ROWS, W, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) /* not 32-bit-only IRAM */))
        return false;
    init_failure = "fft";
    if (!spectrum_fft_init()) return false;
    if (!touch_init() || !lcd_init()) return false;
    present = true;
    init_failure = NULL;
    render(0, false);
    gpio_set_level(PIN_LCD_BL, 1);
    line_start = esp_timer_get_time();
    return true;
}

void cyd_step(unsigned center_khz, bool host_active, cyd_acquire_fn acquire, cyd_retune_fn retune) {
    static bool host_shown;
    if (!present) return;
    if (host_active) {
        if (!host_shown) {
            render(center_khz, true);
            host_shown = true;
            window_n = 0; /* a host SPEC run reuses the spectrum workspace */
        }
        return;
    }
    if (host_shown) {
        host_shown = false;
        reset_display_history();
        line_start = esp_timer_get_time();
    }
    if (cfg_test) return; /* pattern stays on screen until CYDTEST 0 */
    poll_controls(center_khz, retune);
    const span_t *s = &spans[span_index];
    unsigned n = s->nfft, blocks = SNAPSHOT_WORDS / n;
    workspace(n);
    const uint32_t *words;
    unsigned elapsed;
    if (acquire(blocks * n, s->rate, s->bandwidth_mhz, &words, &elapsed)) process_snapshot(words, n, blocks);
    int64_t now = esp_timer_get_time();
    if (now - line_start >= LINE_US && line_ffts) {
        line_start = now;
        finish_line();
        render(center_khz, false);
    }
}

bool cyd_command(const char *line) {
    char reply[256];
    if (!strcmp(line, "CYD?")) {
        int used = present ? snprintf(reply, sizeof(reply), "CYD PRESENT")
                           : snprintf(reply, sizeof(reply), "CYD ABSENT %s", init_failure);
        snprintf(
            reply + used, sizeof(reply) - used,
            " madctl=%02x invert=%u test=%u span=%u knob=%s det=%s lines=%u floor=%d peak=%d/%d heap=%u/%u\n",
            cfg_madctl, cfg_invert, cfg_test, spans[span_index].span_mhz_x10, knob_span ? "SPAN" : "TUNE",
            average ? "AVG" : "PK", lines, floor_q4 >> 4, peak_offset_hz, peak_db,
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        burst_serial_send(reply, strlen(reply));
        return true;
    }
    unsigned madctl_value, invert, test;
    char extra;
    if (sscanf(line, "CYDCFG %x %u %c", &madctl_value, &invert, &extra) == 2 && madctl_value < 256 &&
        invert < 2) {
        cfg_madctl = madctl_value;
        cfg_invert = invert;
        config_save();
        if (present) {
            panel_setup();
            render(0, false);
        }
        burst_serial_send("OK\n", 3);
        return true;
    }
    if (sscanf(line, "CYDTEST %u %c", &test, &extra) == 1 && test < 2) {
        cfg_test = test;
        config_save();
        if (present) render(0, false);
        burst_serial_send("OK\n", 3);
        return true;
    }
    if (!strcmp(line, "CYDLINE")) {
        snprintf(reply, sizeof(reply), "CYDLINE %d", floor_q4 >> 4);
        burst_serial_send(reply, strlen(reply));
        for (unsigned x = 0; x < W; x++) {
            snprintf(reply, sizeof(reply), " %u", column[x]);
            burst_serial_send(reply, strlen(reply));
        }
        burst_serial_send("\n", 1);
        return true;
    }
    return false;
}

bool cyd_passive_command(const char *line) {
    char reply[96];
    if (!strcmp(line, "CYDTOUCH?")) {
        int sx = present ? touch_screen_x() : -1;
        snprintf(reply, sizeof(reply), "CYDTOUCH %d raw=%d,%d z=%d irq=%d boot=%d\n", sx, touch_raw_x,
                 touch_raw_y, touch_raw_z, gpio_get_level(PIN_TP_IRQ), gpio_get_level(PIN_BOOT));
        burst_serial_send(reply, strlen(reply));
        return true;
    }
    return false;
}
