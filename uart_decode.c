/*
 * UART auto-decoder: detects polarity and baud rate from the captured
 * waveform, then decodes 8N1 frames. Pure C, host-testable.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "logic_decode.h"

/*- Definitions -------------------------------------------------------------*/
#define MIN_RUNS        6     // fewer transitions than this = not UART
#define MIN_BIT_SAMPLES 3     // undersampled below this, decode is unreliable
#define MAX_FRAME_BITS  10    // start + 8 data + stop

/*- Variables ---------------------------------------------------------------*/
static const int g_standard_bauds[] =
{
  4800, 9600, 19200, 38400, 57600, 115200, 230400,
  460800, 921600, 1000000, 1500000, 2000000,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline int sample_at(const uint8_t *data, int size, int offset, int i)
{
  int index = offset + i;

  if (index >= size)
    index -= size;

  return data[index];
}

//-----------------------------------------------------------------------------
int uart_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int *run_len = scratch->len;
  int *run_pos = scratch->pos;
  uint8_t *run_lvl = scratch->lvl;
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_UART;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < MIN_RUNS)
    return 0;

  // Idle polarity: the level of the longest run (UART idles between frames)
  int longest = 0;

  for (int r = 1; r < runs; r++)
  {
    if (run_len[r] > run_len[longest])
      longest = r;
  }

  out->idle_high = (run_lvl[longest] != 0);

  // Bit time, first estimate: the shortest run (a lone bit), guarded
  // against a noise spike by requiring a second run of similar width
  int tmin = run_len[0];

  for (int r = 1; r < runs; r++)
  {
    if (run_len[r] < tmin)
      tmin = run_len[r];
  }

  if (tmin < MIN_BIT_SAMPLES)
    return 0;

  int near = 0;

  for (int r = 0; r < runs; r++)
  {
    if (run_len[r] <= tmin + tmin / 4)
      near++;
  }

  if (near < 2)
    return 0;

  // Refine: every run should be an integer number of bits; average the
  // implied bit width over all runs that fit a UART frame
  int64_t width_sum = 0;
  int bit_count = 0;

  for (int r = 0; r < runs; r++)
  {
    int bits = (run_len[r] + tmin / 2) / tmin;

    if (bits < 1 || bits > MAX_FRAME_BITS)
      continue; // idle gap or glitch: not part of the estimate

    width_sum += run_len[r];
    bit_count += bits;
  }

  if (bit_count < MIN_RUNS)
    return 0;

  // bit time in samples, x256 fixed point
  int bit_x256 = (int)(width_sum * 256 / bit_count);

  // Plausibility: real UART traffic idles somewhere in a record this long.
  // A continuous bitstream with no run of at least a frame's worth of idle
  // (a clock, WS2812 data, a random bus) is not UART.
  if ((int64_t)run_len[longest] * 256 < (int64_t)bit_x256 * (MAX_FRAME_BITS + 1))
    return 0;

  // Decode 8N1: idle -> start bit (non-idle) -> 8 data bits LSB-first ->
  // stop bit (idle). The run list locates frame starts; the bits themselves
  // are sampled directly at their centers (x256 fixed-point positions).
  int idle = out->idle_high ? 1 : 0;

  for (int r = 0; r < runs - 1 && out->count < LOGIC_MAX_BYTES; r++)
  {
    // A frame starts where a non-idle run follows an idle run
    if (run_lvl[r] != idle)
      continue;

    int64_t frame = (int64_t)run_pos[r + 1] * 256;
    int value = 0;
    bool ok = true;

    // Data bit n center = frame + (1.5 + n) * bit
    for (int b = 0; b < 8; b++)
    {
      int64_t center = frame + ((int64_t)bit_x256 * (3 + 2 * b)) / 2;
      int si = (int)(center / 256);

      if (si >= size)
      {
        ok = false;
        break;
      }

      int bitv = (sample_at(data, size, offset, si) > mid) ? 1 : 0;

      if (!out->idle_high)
        bitv = !bitv; // inverted logic: mark = low

      if (bitv)
        value |= (1 << b);
    }

    if (!ok)
      break;

    // Stop bit must be at idle level
    int64_t stop_center = frame + ((int64_t)bit_x256 * 19) / 2;
    int stop_i = (int)(stop_center / 256);

    if (stop_i < size)
    {
      int sv = (sample_at(data, size, offset, stop_i) > mid) ? 1 : 0;

      if (sv == idle)
      {
        out->bytes[out->count] = (uint8_t)value;
        out->pos[out->count] = run_pos[r + 1];
        out->end[out->count] =
            (int)((frame + (int64_t)bit_x256 * MAX_FRAME_BITS) / 256);
        out->count++;
      }
      else
      {
        out->errors++;
      }
    }

    // Resume the scan at the first run starting inside or after the stop
    // bit, so this frame's data runs are not mistaken for new frames
    int frame_end = (int)((frame + (int64_t)bit_x256 * (MAX_FRAME_BITS - 1)) / 256);

    while (r + 1 < runs && run_pos[r + 1] < frame_end)
      r++;

    r--; // the for-loop increment lands on the idle run before the next frame
  }

  if (out->count == 0)
    return 0;

  // Baud from the refined bit time, snapped to a standard rate within 4%
  int64_t bit_ns = (int64_t)bit_x256 * period_ns / 256;

  if (bit_ns <= 0)
    return 0;

  int baud = (int)(1000000000ll / bit_ns);

  out->rate = baud;

  for (unsigned i = 0; i < sizeof(g_standard_bauds) / sizeof(g_standard_bauds[0]); i++)
  {
    int std = g_standard_bauds[i];
    int diff = baud - std;

    if (diff < 0)
      diff = -diff;

    if ((int64_t)diff * 100 <= (int64_t)std * 4)
    {
      out->rate = std;
      break;
    }
  }

  snprintf(out->info, sizeof(out->info), "UART %d 8N1%s err %d",
      out->rate, out->idle_high ? "" : " inv", out->errors);

  return out->count;
}
