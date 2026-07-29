/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * fun-open-5012h hardware emulator: shared declarations.
 *
 * The emulator executes the UNMODIFIED firmware image - the same open-5012h.bin
 * that gets flashed to the device - on a Cortex-M4 core (Unicorn Engine) and
 * models the FNIRSI 5012H board around it: the GD32F407 peripherals the
 * firmware drives, the AD9288 dual ADC behind them, the ST7789 panel on its
 * bit-banged 8080 bus, the two button encoders and the battery. No firmware
 * source knows this exists; the boundary is the silicon, not the API.
 */

#ifndef _EMU_H_
#define _EMU_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*- Memory map: linker/gd32f407ve.ld and the GD32F407VE datasheet ------------*/
#define EMU_FLASH_BASE   0x08000000u
#define EMU_FLASH_SIZE   (512 * 1024)
#define EMU_TCM_BASE     0x10000000u
#define EMU_TCM_SIZE     (64 * 1024)
#define EMU_SRAM_BASE    0x20000000u
#define EMU_SRAM_SIZE    (128 * 1024)
#define EMU_PERIPH_BASE  0x40000000u
#define EMU_PERIPH_SIZE  0x00030000u     /* APB1 + APB2 + AHB1 blocks in use */
#define EMU_SCS_BASE     0xE000E000u     /* SysTick, NVIC, SCB               */
#define EMU_SCS_SIZE     0x1000u
#define EMU_SIG_BASE     0x1FFF7000u     /* device electronic signature page */
#define EMU_SIG_SIZE     0x1000u

#define EMU_CPU_HZ       250000000ull    /* what main.c's PLL sets up        */
#define DMA1_CH2_IRQN    58              /* DMA1_Channel2, see gd32f4xx.h    */

/* The settings store: the last flash sector, which config.c owns */
#define EMU_CFG_OFFSET   (384 * 1024)
#define EMU_CFG_SIZE     (128 * 1024)

/*- cpu.c: the core and its memory ------------------------------------------*/
extern uint8_t *emu_flash;               /* 512 KB, host-side backing store  */
extern uint8_t *emu_tcm;
extern uint8_t *emu_sram;

bool cpu_init(void);
void cpu_load_image(const uint8_t *image, size_t size);
void cpu_reset(void);
/* Executes one slice; advances virtual time. Returns false when the core has
 * wedged in a way the board cannot resolve. */
bool cpu_step(void);
uint64_t cpu_now_ns(void);
uint32_t cpu_pc(void);
bool cpu_in_thread_mode(void);
bool cpu_irq_masked(void);
void cpu_enter_irq(int irqn);
uint64_t cpu_instructions(void);
/* One line describing where the core is, for hang reports */
void cpu_describe(char *buf, size_t size);

/*- board.c: the GD32F407 peripherals and the board around them -------------*/
void board_init(void);
void board_reset(void);
uint64_t board_mmio_read(uint64_t offset, unsigned size);
void board_mmio_write(uint64_t offset, unsigned size, uint64_t value);
uint64_t scs_mmio_read(uint64_t offset, unsigned size);
void scs_mmio_write(uint64_t offset, unsigned size, uint64_t value);
/* Applies a guest store into flash with NOR semantics; false = rejected */
bool board_flash_write(uint32_t addr, uint64_t value, unsigned size);
void board_sync(void);                   /* catch peripherals up to cpu time */
int  board_pending_irq(void);            /* -1 when none is ready to take    */
void board_irq_accepted(int irqn);
bool board_reset_requested(void);
/* One line describing the acquisition, for hang reports */
void board_describe(char *buf, size_t size);
int board_late_prescaler_latches(void);

/* Battery, in millivolts. Read back through the modeled ADC0. */
extern int emu_battery_mv;
extern bool emu_battery_charging;

/* Buttons, as the bit mask buttons.h defines (BTN_STOP, BTN_MENU, ...).
 * board.c drives the two priority encoders from it, exactly as the panel
 * wiring does, so buttons_read() decodes it without knowing the difference. */
void board_set_buttons(uint32_t mask);
uint32_t board_get_buttons(void);

/*- The analog front end, as decoded from the pins the firmware drives ------*/
typedef struct
{
  int  relay_scale;      /* 0..7 = 50 mV .. 10 V/div, -1 = none selected     */
  int  dac_code;         /* DAC0_R12DH, 0..4095: the vertical offset         */
  bool ac_coupling;      /* AC_DC pin low                                    */
  int  backlight_pct;    /* from the BL pin / TIMER2 PWM duty                */
} AfeState;
void board_get_afe(AfeState *out);

/*- st7789.c: the panel on the far side of the 8080 bus ---------------------*/
#define LCD_W 320
#define LCD_H 240
void st7789_init(void);
void st7789_reset_pin(bool level);
/* One bus cycle: the byte on PE[7:0] latched by the WR rising edge, with the
 * RS pin telling command from data. */
void st7789_bus_write(uint8_t value, bool is_command);
extern uint16_t st7789_fb[LCD_W * LCD_H];  /* RGB565 as the panel holds it   */
extern uint64_t st7789_writes;             /* pixel writes, for the frame tick */
/* Bumped each time a left-to-right run of single-column writes wraps back to
 * the left, i.e. the instant a column sweep finishes and the panel holds one
 * coherent frame. Screenshots wait for this. */
extern uint64_t st7789_sweep_end;
extern bool st7789_display_on;

/*- siggen.c: what is on the probe, and what the instrument does to it ------*/
void sig_init(void);
/* The source: "square f=10k vpp=2 duty=30 rise=20n noise=10m" */
bool sig_configure(const char *spec, char *err, size_t errsz);
/* The instrument's imperfections: "bw=100M jitter=2p dnl=0.35 probe=10" */
bool sig_configure_afe(const char *spec, char *err, size_t errsz);
const char *sig_describe(void);
const char *sig_describe_afe(void);
/* Volts at the probe tip, in microvolts, at time t */
int64_t sig_probe_uv(uint64_t t_ns);
/* One AD9288 converter's output byte for the instant t */
uint8_t sig_adc_sample(uint64_t t_ns, const AfeState *afe, int converter);

/*- video.c -----------------------------------------------------------------*/
/* scale 0 picks one that fits; bare drops the drawn front panel and shows the
 * screen on its own, the way this used to look */
bool video_init(int scale, bool headless, bool bare, const char *title);
bool video_pump(void);                  /* false when the window was closed  */
void video_present(void);
void video_shutdown(void);
bool video_save_png(const char *path, int scale);

/*- main.c ------------------------------------------------------------------*/
void emu_log(const char *fmt, ...);
extern bool emu_verbose;

#endif /* _EMU_H_ */
