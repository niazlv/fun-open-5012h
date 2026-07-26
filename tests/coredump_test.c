/*
 * Host-side tests for the coredump ring in debug_coredump.c.
 *
 * The point of these tests is the one property the feature stands or falls
 * on: a dump written just before the device stops has to still be there, and
 * be recognised as valid, after the reset that follows. On the device the
 * ring lives in a block of SRAM that nothing is linked into, so the startup
 * code neither loads nor zeroes it; here that block is a plain array and a
 * "reset" is simply calling debug_coredump_init() again without touching it.
 *
 * debug_coredump.c is included directly so the tests can reach the static
 * ring helpers. tests/run_coredump_test.sh stages an unmodified copy of it
 * next to the stub device headers, which is what lets those stubs stand in
 * for lcd.h, timer.h and friends.
 *
 * Build & run, from the repository root:
 *   sh tests/run_coredump_test.sh
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "gd32f4xx.h"   // the stub in tests/hoststub
#include "lcd.h"        // ditto
#include "ui.h"         // the real ones: no device dependencies
#include "menu_widget.h"

/*- Device stubs ------------------------------------------------------------*/
unsigned char g_coredump_fake_ram[3 * 1024] __attribute__((aligned(8)));

scb_stub_t g_scb_stub;

const Font *FONT_SMALL = NULL;
const Font *FONT_LARGE = NULL;

static uint32_t g_fake_ms = 0;

void lcd_fill_rect(int x, int y, int w, int h, int color)
{ (void)x; (void)y; (void)w; (void)h; (void)color; }
void lcd_draw_rect(int x, int y, int w, int h, int color)
{ (void)x; (void)y; (void)w; (void)h; (void)color; }
void lcd_set_font(const Font *font) { (void)font; }
void lcd_set_color(int bg, int fg) { (void)bg; (void)fg; }
void lcd_puts(int x, int y, const char *str) { (void)x; (void)y; (void)str; }

void timer_add(int *timer) { (void)timer; }
void timer_remove(int *timer) { (void)timer; }
uint32_t timer_ms(void) { return g_fake_ms; }

void ui_pop(void) {}
void ui_push(const ui_screen_t *screen, void *ctx) { (void)screen; (void)ctx; }
void ui_request_redraw(void) {}
void menu_close_popups(void) {}
void menu_open_info(const info_page_t *page) { (void)page; }
void menu_action_info(const void *arg) { (void)arg; }

// Linker symbols the unwinder bounds itself with. The values only have to be
// a plausible, ordered TCM range: the tests never hand it a stack pointer
// inside it, because dereferencing a truncated 32-bit address on a 64-bit
// host would not be a test of anything.
char _end __attribute__((aligned(4)));
char _stack_top __attribute__((aligned(4)));
uint32_t __etext;

#define FAKE_TCM_LO   0x10008000ul
#define FAKE_TCM_HI   0x10010000ul
#define FAKE_TEXT_HI  0x08030000ul

/*- Code under test ---------------------------------------------------------*/
// Resolved from the staging directory, see tests/run_coredump_test.sh
#include "debug_coredump.c"

/*- Test harness ------------------------------------------------------------*/
static int g_failures = 0;

static void check(const char *name, bool ok)
{
  printf("  %s %s\n", ok ? "PASS" : "FAIL", name);

  if (!ok)
    g_failures++;
}

static void check_int(const char *name, long got, long want)
{
  bool ok = (got == want);

  printf("  %s %-46s got %8ld  want %8ld\n", ok ? "PASS" : "FAIL", name, got,
      want);

  if (!ok)
    g_failures++;
}

//-----------------------------------------------------------------------------
// A dump the way error() records one: a message and a plausible PC. The stack
// pointer is deliberately outside the fake TCM range so capture_stack_trace
// records frame 0 and then stops instead of dereferencing a host address.
static void capture(const char *message, uint32_t pc)
{
  g_fake_ms += 1000;
  debug_coredump_capture(ERROR_TYPE_GENERAL_ERROR, message, pc,
      0x08001235u /* thumb return address */, 0 /* implausible sp */);
}

//-----------------------------------------------------------------------------
static void fill_with_garbage(void)
{
  for (size_t i = 0; i < sizeof(g_coredump_fake_ram); i++)
    g_coredump_fake_ram[i] = (unsigned char)(0x5A + i * 31);
}

//-----------------------------------------------------------------------------
int main(void)
{
  // Order the fake bounds the way the linker does on the device
  memset(&g_scb_stub, 0, sizeof(g_scb_stub));
  __etext = FAKE_TEXT_HI;

  printf("\ncold start over uninitialised SRAM\n");
  fill_with_garbage();
  debug_coredump_init();
  check("garbage is not mistaken for a ring", store_is_valid());
  check_int("count", g_store->count, 0);
  check_int("boot id", g_store->boot_id, 1);
  check("ring not reported as retained", !debug_coredump_ring_retained());

  printf("\ncapturing dumps\n");
  capture("first failure", 0x08001000u);
  capture("second failure", 0x08002000u);
  check_int("count", g_store->count, 2);
  check_int("write index", g_store->write_index, 2);
  check("ring still validates", store_is_valid());
  check_int("entry 0 boot id", entry_at(0)->boot_id, 1);
  check("entry 0 is the oldest",
      0 == strcmp(entry_at(0)->message, "first failure"));
  check("entry 1 is the newest",
      0 == strcmp(entry_at(1)->message, "second failure"));

  // What the unwinder accepts as a code address is decided by the addresses
  // of the linker symbols, which cannot be faked on a 64-bit host: those
  // paths are target-only. What is checked here is the property that keeps
  // the fault handler alive, further down: an implausible stack pointer must
  // stop the scan rather than start dereferencing.

  printf("\nreset: init runs again over the untouched block\n");
  debug_coredump_init();
  check("ring survives", store_is_valid());
  check_int("dumps still there", g_store->count, 2);
  check_int("boot id advanced", g_store->boot_id, 2);
  check("reported as retained", debug_coredump_ring_retained());
  check("older dump still readable",
      0 == strcmp(entry_at(0)->message, "first failure"));
  check_int("older dump remembers its boot", entry_at(0)->boot_id, 1);

  printf("\na dump from the new boot is tagged with the new boot id\n");
  capture("after the reset", 0x08003000u);
  check_int("count", g_store->count, 3);
  check_int("new entry boot id", entry_at(2)->boot_id, 2);
  check("previous boot still distinguishable",
      entry_at(0)->boot_id != g_store->boot_id);

  printf("\ncorruption is detected\n");
  g_coredump_fake_ram[sizeof(coredump_store_t) / 2] ^= 0xFF;
  check("crc rejects the ring", !store_is_valid());
  debug_coredump_init();
  check("init rebuilt a clean ring", store_is_valid());
  check_int("count reset", g_store->count, 0);
  check_int("boot id restarted", g_store->boot_id, 1);

  printf("\na stale ring from another firmware layout is rejected\n");
  g_store->magic = COREDUMP_MAGIC ^ 1;
  store_seal();
  check("magic mismatch rejected", !store_is_valid());
  debug_coredump_init();
  check_int("count reset", g_store->count, 0);

  printf("\nring wraps and keeps the newest entries\n");
  for (int i = 0; i < MAX_COREDUMP_ENTRIES + 2; i++)
  {
    char msg[32];

    snprintf(msg, sizeof(msg), "dump %d", i);
    capture(msg, 0x08004000u + i);
  }

  check_int("count saturates", g_store->count, MAX_COREDUMP_ENTRIES);
  check("oldest surviving entry is the third one written",
      0 == strcmp(entry_at(0)->message, "dump 2"));
  check("newest entry is the last one written",
      0 == strcmp(entry_at(MAX_COREDUMP_ENTRIES - 1)->message,
          "dump 7"));
  check("ring still validates after wrapping", store_is_valid());

  printf("\nring survives a reset while full\n");
  debug_coredump_init();
  check_int("count", g_store->count, MAX_COREDUMP_ENTRIES);
  check("oldest entry unchanged",
      0 == strcmp(entry_at(0)->message, "dump 2"));

  printf("\nclearing\n");
  debug_coredump_clear_all();
  check_int("count", g_store->count, 0);
  check("still a valid ring", store_is_valid());
  debug_coredump_init();
  check_int("empty ring survives a reset", g_store->count, 0);

  printf("\nunwinder rejects an implausible stack pointer\n");
  {
    stack_frame_t trace[MAX_STACK_TRACE_DEPTH];
    int depth = 12345;

    // lr is not a code address either: nothing at all should be recorded
    capture_stack_trace(trace, &depth, 0, 0);
    check_int("depth", depth, 0);
  }

  printf("\nring fits the retained block\n");
  check_int("sizeof(coredump_store_t)", (long)sizeof(coredump_store_t),
      (long)sizeof(coredump_store_t));
  check("fits", sizeof(coredump_store_t) <= COREDUMP_RAM_SIZE);

  printf("\n%s (%d failures)\n\n", g_failures ? "FAILURES" : "ALL PASSED",
      g_failures);

  return g_failures ? 1 : 0;
}
