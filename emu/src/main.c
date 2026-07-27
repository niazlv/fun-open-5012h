/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Emulator entry point: load a firmware image, run the board, show the panel.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#include "emu.h"

/*- Definitions -------------------------------------------------------------*/
/* Slices between window updates. One slice is 256 instructions, so this is
 * about a millisecond of guest time per repaint - fast enough to feel live,
 * infrequent enough that the repaint is not the bottleneck. */
#define SLICES_PER_FRAME   4096

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  const char *name;     /* the key to press                                 */
  uint32_t    btn;
} NamedButton;

/*- Variables ---------------------------------------------------------------*/
bool emu_verbose;

static const NamedButton g_named[] =
{
  { "stop",  1u << 0  }, { "up",    1u << 1  }, { "edge",  1u << 2  },
  { "mode",  1u << 3  }, { "f1",    1u << 4  }, { "right", 1u << 5  },
  { "acdc",  1u << 6  }, { "auto",  1u << 7  }, { "50",    1u << 8  },
  { "menu",  1u << 9  }, { "trigup",1u << 10 }, { "left",  1u << 11 },
  { "trigdn",1u << 12 }, { "save",  1u << 13 }, { "trig",  1u << 14 },
  { "down",  1u << 15 }, { "f2",    1u << 16 }, { "shift", 1u << 17 },
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void emu_log(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  fputs("[emu] ", stderr);
  vfprintf(stderr, fmt, ap);
  fputc('\n', stderr);
  va_end(ap);
}

//-----------------------------------------------------------------------------
static void usage(void)
{
  printf(
    "fun-open-5012h hardware emulator\n"
    "\n"
    "Runs an unmodified GD32F407 firmware image on an emulated FNIRSI 5012H.\n"
    "\n"
    "usage: emu [options] <firmware.bin>\n"
    "\n"
    "  --signal <spec>    what is on the probe (default: sine f=1k vpp=1)\n"
    "  --afe <spec>       the instrument's own imperfections\n"
    "  --scale N          window magnification (default 3)\n"
    "  --headless         no window; use with --script\n"
    "  --script <file>    run a key script, then exit\n"
    "  --shot <file.png>  screenshot on exit\n"
    "  --run-ms N         stop after N ms of emulated time\n"
    "  --battery MV       pack voltage in millivolts (default 3950)\n"
    "  --charging         report the charger as connected\n"
    "  --flash <file>     persist the whole flash image here across runs\n"
    "  --dump-sram <file> write the 128 KB of main SRAM out at exit\n"
    "  --heartbeat MS     report speed and frame timing every MS of host time\n"
    "  --watchdog MS      report a hang if nothing is drawn for MS of\n"
    "                     emulated time; exits non-zero if one was seen\n"
    "  --verbose          log peripheral events\n"
    "\n"
    "signal spec: <wave> [key=value ...]\n"
    "  waves : dc sine square triangle saw pulse noise uart glitch am burst rc\n"
    "  keys  : f= amp= vpp= ofs= duty= rise= noise= phase= baud= data=\n"
    "          bytes= idle= text= every= width= modf= depth= cycles=\n"
    "  values take engineering suffixes: 1k 2.5M 500m 20n\n"
    "  text= takes the rest of the spec verbatim (UTF-8 ok): put it last\n"
    "\n"
    "  e.g. --signal \"square f=10k vpp=2 duty=30 rise=20n noise=10m\"\n"
    "       --signal \"uart baud=115200 vpp=3.3 text=Hi there\"\n"
    "       --signal \"glitch f=100k every=32 width=60n\"\n"
    "\n"
    "front-end spec: [ideal|real] [key=value ...]\n"
    "  bw=        analog bandwidth, which is what rounds edges (default 100M)\n"
    "  overshoot= amplifier peaking after an edge, percent (default 4)\n"
    "  jitter=    ADC aperture jitter, seconds rms (default 2p)\n"
    "  dnl=       converter code nonlinearity, LSB rms (default 0.35)\n"
    "  adcnoise=  converter noise floor, LSB rms (default 0.35)\n"
    "  ilgain=    gain mismatch between the interleaved halves, %% (default 0.8)\n"
    "  iloffset=  and their offset mismatch, LSB (default 5)\n"
    "  probe=     1 or 10 for a 1x or 10x probe (default 1)\n"
    "  ideal      turn every impairment above off, for comparison\n"
    "\n"
    "  e.g. --afe \"bw=20M overshoot=12\"   a soft, ringing front end\n"
    "       --afe ideal                    a perfect instrument\n"
    "\n"
    "script lines: press <key>[+<key>...] | hold <key> | release <key>\n"
    "              wait <ms> | shot <file.png> | signal <spec> | echo <text>\n"
    "  keys: stop up edge mode f1 right acdc auto 50 menu trigup left\n"
    "        trigdn save trig down f2 shift\n"
    "\n"
    "window keys: arrows, space=STOP, a=AUTO, m=MODE, e=EDGE, c=AC/DC,\n"
    "             t=TRIG, s=SAVE, 5=50%%, enter=MENU, F1, F2, shift, F12=shot\n");
}

//-----------------------------------------------------------------------------
static uint32_t lookup_button(const char *name)
{
  for (unsigned i = 0; i < sizeof(g_named) / sizeof(g_named[0]); i++)
  {
    if (0 == strcmp(g_named[i].name, name))
      return g_named[i].btn;
  }

  emu_log("script: unknown key '%s'", name);

  return 0;
}

//-----------------------------------------------------------------------------
static double host_seconds(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);

  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

//-----------------------------------------------------------------------------
// Progress is measured by what reaches the panel, because that is the one
// thing a working instrument cannot stop doing. Two separate signals: pixels
// (the firmware is drawing something) and completed sweeps (it is drawing
// whole traces). A hang shows up as the first one stopping.
static uint64_t g_last_writes;
static uint64_t g_last_sweeps;
static uint64_t g_stall_since_ns;
static uint64_t g_frames;
static uint64_t g_frame_sum_ns;
static uint64_t g_frame_worst_ns;
static uint64_t g_frame_mark_ns;
static double g_start_host;
static double g_next_heartbeat;
static bool g_hang_reported;

static int g_clock_countdown;
static int g_heartbeat_ms;      /* 0 = off */
static int g_watchdog_ms;       /* 0 = off */
static bool g_hang_seen;

//-----------------------------------------------------------------------------
static void report_hang(const char *why)
{
  char cpu[160], board[224];

  cpu_describe(cpu, sizeof(cpu));
  board_describe(board, sizeof(board));

  emu_log("WATCHDOG: %s", why);
  emu_log("  core:  %s", cpu);
  emu_log("  board: %s", board);
  emu_log("  after %.1f ms emulated, %.1f s host, %llu frames",
      cpu_now_ns() / 1e6, host_seconds() - g_start_host,
      (unsigned long long)g_frames);

  g_hang_seen = true;
  g_hang_reported = true;
}

//-----------------------------------------------------------------------------
static void progress_check(void)
{
  uint64_t now = cpu_now_ns();

  if (st7789_sweep_end != g_last_sweeps)
  {
    uint64_t dt = now - g_frame_mark_ns;

    g_last_sweeps = st7789_sweep_end;
    g_frame_mark_ns = now;
    g_frames++;
    g_frame_sum_ns += dt;

    if (dt > g_frame_worst_ns)
      g_frame_worst_ns = dt;
  }

  if (st7789_writes != g_last_writes)
  {
    g_last_writes = st7789_writes;
    g_stall_since_ns = now;
    g_hang_reported = false;
  }
  else if (g_watchdog_ms > 0 && !g_hang_reported &&
      now - g_stall_since_ns > (uint64_t)g_watchdog_ms * 1000000ull)
  {
    char why[96];

    snprintf(why, sizeof(why), "nothing drawn for %d ms of emulated time",
        g_watchdog_ms);
    report_hang(why);
  }

  // Reading the host clock is a system call, and this runs once per slice -
  // once per microsecond of emulated time. Sampling it every few thousand
  // slices keeps the heartbeat from costing more than it reports.
  if (g_heartbeat_ms > 0 && ++g_clock_countdown >= 4096)
  {
    double host = host_seconds();

    g_clock_countdown = 0;

    if (host >= g_next_heartbeat)
    {
      double elapsed = host - g_start_host;

      g_next_heartbeat = host + g_heartbeat_ms / 1000.0;

      emu_log("hb: %8.1f ms emulated | %6.2f s host (%.2fx) | %5.1f MIPS | "
          "%llu frames, avg %.1f ms, worst %.1f ms",
          cpu_now_ns() / 1e6, elapsed,
          elapsed > 0 ? (cpu_now_ns() / 1e9) / elapsed : 0.0,
          elapsed > 0 ? cpu_instructions() / 1e6 / elapsed : 0.0,
          (unsigned long long)g_frames,
          g_frames ? g_frame_sum_ns / 1e6 / (double)g_frames : 0.0,
          g_frame_worst_ns / 1e6);
    }
  }
}

//-----------------------------------------------------------------------------
// One step of the whole board: the core, the peripherals it drives, and the
// interrupt it may be owed.
static bool emu_tick(void)
{
  int irq;

  if (!cpu_step())
    return false;

  board_sync();

  // The core takes a pending interrupt between instructions, when it is not
  // already in a handler and has not masked them
  irq = board_pending_irq();

  if (irq >= 0 && cpu_in_thread_mode() && !cpu_irq_masked())
  {
    board_irq_accepted(irq);
    cpu_enter_irq(irq);
  }

  if (board_reset_requested())
  {
    emu_log("firmware requested a system reset");
    board_reset();
    cpu_reset();
  }

  progress_check();

  return true;
}

//-----------------------------------------------------------------------------
// Runs the board for the given amount of EMULATED time, repainting as it goes.
static bool run_for_ms(int ms, bool present)
{
  uint64_t target = cpu_now_ns() + (uint64_t)ms * 1000000ull;
  int since_frame = 0;

  while (cpu_now_ns() < target)
  {
    if (!emu_tick())
      return false;

    if (++since_frame >= SLICES_PER_FRAME)
    {
      since_frame = 0;

      if (present)
      {
        video_present();

        if (!video_pump())
          return false;
      }
    }
  }

  return true;
}

//-----------------------------------------------------------------------------
// The scope paints its trace one column per pass of the main loop, so at most
// instants the panel holds part of one sweep and part of the previous one -
// which is what makes a trace look torn or broken in a still frame. Waiting
// for a sweep to finish before capturing gets a whole coherent frame instead.
// Screens that do not sweep (menus, the launcher) simply time out, and the
// wait costs nothing.
static void run_to_sweep_end(int max_ms)
{
  uint64_t mark = st7789_sweep_end;
  uint64_t target = cpu_now_ns() + (uint64_t)max_ms * 1000000ull;

  while (cpu_now_ns() < target)
  {
    if (!emu_tick())
      return;

    if (st7789_sweep_end != mark)
      return;
  }
}

//-----------------------------------------------------------------------------
static bool run_script(const char *path)
{
  FILE *f = fopen(path, "r");
  char line[512];

  if (!f)
  {
    emu_log("cannot open script %s", path);
    return false;
  }

  while (fgets(line, sizeof(line), f))
  {
    char *nl = strchr(line, '\n');
    char *cmd, *arg;

    if (nl)
      *nl = 0;

    if (line[0] == '#' || line[0] == 0)
      continue;

    cmd = line;

    while (*cmd == ' ' || *cmd == '\t')
      cmd++;

    arg = strpbrk(cmd, " \t");

    if (arg)
    {
      *arg++ = 0;

      while (*arg == ' ' || *arg == '\t')
        arg++;
    }
    else
    {
      arg = (char *)"";
    }

    if (0 == strcmp(cmd, "wait"))
    {
      if (!run_for_ms(atoi(arg), true))
        break;
    }
    else if (0 == strcmp(cmd, "press") || 0 == strcmp(cmd, "hold"))
    {
      uint32_t mask = 0;
      char *save = NULL;

      for (char *t = strtok_r(arg, "+", &save); t; t = strtok_r(NULL, "+", &save))
        mask |= lookup_button(t);

      board_set_buttons(board_get_buttons() | mask);

      if (0 == strcmp(cmd, "press"))
      {
        // Long enough to clear the 20 ms debounce in buttons.c, short enough
        // not to trip the 250 ms auto-repeat
        if (!run_for_ms(60, true))
          break;

        board_set_buttons(board_get_buttons() & ~mask);

        if (!run_for_ms(60, true))
          break;
      }
    }
    else if (0 == strcmp(cmd, "release"))
    {
      uint32_t mask = 0;
      char *save = NULL;

      for (char *t = strtok_r(arg, "+", &save); t; t = strtok_r(NULL, "+", &save))
        mask |= lookup_button(t);

      board_set_buttons(board_get_buttons() & ~mask);

      if (!run_for_ms(60, true))
        break;
    }
    else if (0 == strcmp(cmd, "shot"))
    {
      run_to_sweep_end(400);

      if (video_save_png(arg, 2))
        emu_log("wrote %s", arg);
      else
        emu_log("failed to write %s", arg);
    }
    else if (0 == strcmp(cmd, "signal"))
    {
      char err[128] = "";

      if (!sig_configure(arg, err, sizeof(err)))
        emu_log("script: bad signal spec: %s", err);
      else
        emu_log("signal: %s", sig_describe());
    }
    else if (0 == strcmp(cmd, "echo"))
    {
      printf("%s\n", arg);
      fflush(stdout);
    }
    else
    {
      emu_log("script: unknown command '%s'", cmd);
    }
  }

  fclose(f);

  return true;
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  const char *fw_path = NULL;
  const char *script = NULL;
  const char *shot = NULL;
  const char *flash_file = NULL;
  const char *dump_sram = NULL;
  int scale = 3;
  bool headless = false;
  int run_ms = 0;
  char err[128] = "";
  FILE *f;
  uint8_t *image;
  long size;

  for (int i = 1; i < argc; i++)
  {
    const char *a = argv[i];

    if (0 == strcmp(a, "--help") || 0 == strcmp(a, "-h"))
    {
      usage();
      return 0;
    }
    else if (0 == strcmp(a, "--signal") && i + 1 < argc)
    {
      if (!sig_configure(argv[++i], err, sizeof(err)))
      {
        fprintf(stderr, "bad --signal: %s\n", err);
        return 1;
      }
    }
    else if (0 == strcmp(a, "--afe") && i + 1 < argc)
    {
      if (!sig_configure_afe(argv[++i], err, sizeof(err)))
      {
        fprintf(stderr, "bad --afe: %s\n", err);
        return 1;
      }
    }
    else if (0 == strcmp(a, "--scale") && i + 1 < argc)
      scale = atoi(argv[++i]);
    else if (0 == strcmp(a, "--headless"))
      headless = true;
    else if (0 == strcmp(a, "--script") && i + 1 < argc)
      script = argv[++i];
    else if (0 == strcmp(a, "--shot") && i + 1 < argc)
      shot = argv[++i];
    else if (0 == strcmp(a, "--run-ms") && i + 1 < argc)
      run_ms = atoi(argv[++i]);
    else if (0 == strcmp(a, "--battery") && i + 1 < argc)
      emu_battery_mv = atoi(argv[++i]);
    else if (0 == strcmp(a, "--charging"))
      emu_battery_charging = true;
    else if (0 == strcmp(a, "--flash") && i + 1 < argc)
      flash_file = argv[++i];
    else if (0 == strcmp(a, "--dump-sram") && i + 1 < argc)
      dump_sram = argv[++i];
    else if (0 == strcmp(a, "--heartbeat") && i + 1 < argc)
      g_heartbeat_ms = atoi(argv[++i]);
    else if (0 == strcmp(a, "--watchdog") && i + 1 < argc)
      g_watchdog_ms = atoi(argv[++i]);
    else if (0 == strcmp(a, "--verbose"))
      emu_verbose = true;
    else if (a[0] == '-')
    {
      fprintf(stderr, "unknown option %s (try --help)\n", a);
      return 1;
    }
    else
    {
      fw_path = a;
    }
  }

  if (!fw_path)
  {
    usage();
    return 1;
  }

  f = fopen(fw_path, "rb");

  if (!f)
  {
    fprintf(stderr, "cannot open %s\n", fw_path);
    return 1;
  }

  fseek(f, 0, SEEK_END);
  size = ftell(f);
  fseek(f, 0, SEEK_SET);

  image = malloc((size_t)size);

  if (!image || fread(image, 1, (size_t)size, f) != (size_t)size)
  {
    fprintf(stderr, "cannot read %s\n", fw_path);
    return 1;
  }

  fclose(f);

  if (!cpu_init())
    return 1;

  sig_init();
  board_init();
  cpu_load_image(image, (size_t)size);
  free(image);

  // The settings store lives in the last flash sector. Restoring it makes the
  // emulated instrument remember its setup and its calibration between runs,
  // the same way the device does.
  if (flash_file)
  {
    FILE *ff = fopen(flash_file, "rb");

    if (ff)
    {
      uint8_t *saved = malloc(EMU_CFG_SIZE);

      if (saved && fread(saved, 1, EMU_CFG_SIZE, ff) == EMU_CFG_SIZE)
      {
        memcpy(emu_flash + EMU_CFG_OFFSET, saved, EMU_CFG_SIZE);
        emu_log("restored settings store from %s", flash_file);
      }

      free(saved);
      fclose(ff);
    }
  }

  cpu_reset();

  emu_log("firmware %s (%ld bytes)", fw_path, size);
  emu_log("signal: %s", sig_describe());
  emu_log("%s", sig_describe_afe());

  g_start_host = host_seconds();
  g_next_heartbeat = g_start_host + g_heartbeat_ms / 1000.0;

  video_init(scale, headless, "fun-open-5012h");

  if (script)
  {
    run_script(script);
  }
  else if (run_ms > 0)
  {
    run_for_ms(run_ms, !headless);
  }
  else
  {
    while (run_for_ms(50, !headless))
      ;
  }

  if (shot)
  {
    if (video_save_png(shot, 2))
      emu_log("wrote %s", shot);
  }

  // The whole of main SRAM: capture ring, storage record and the spare block,
  // for looking at what the acquisition actually put there
  if (dump_sram)
  {
    FILE *df = fopen(dump_sram, "wb");

    if (df)
    {
      fwrite(emu_sram, 1, EMU_SRAM_SIZE, df);
      fclose(df);
      emu_log("wrote %s (%d bytes of SRAM)", dump_sram, EMU_SRAM_SIZE);
    }
  }

  if (flash_file)
  {
    FILE *ff = fopen(flash_file, "wb");

    if (ff)
    {
      fwrite(emu_flash + EMU_CFG_OFFSET, 1, EMU_CFG_SIZE, ff);
      fclose(ff);
    }
  }

  emu_log("ran %.1f ms of emulated time (%llu instructions)",
      cpu_now_ns() / 1e6, (unsigned long long)cpu_instructions());

  video_shutdown();

  // A run that hit the watchdog reports it in its exit status, so a script
  // that captures screenshots fails loudly instead of writing a frozen frame
  return g_hang_seen ? 2 : 0;
}
