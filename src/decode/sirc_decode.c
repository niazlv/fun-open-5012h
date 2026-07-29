/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Sony SIRC - the infrared remote on every Sony television, amplifier and
 * disc player, and the shape Samsung and RCA borrowed. Seen here after a
 * TSOP-style demodulator, so the line rests HIGH and a mark - carrier
 * present - pulls it low.
 *
 * Everything is measured in T, nominally 600 us:
 *
 *   leader  4T mark, 1T space
 *   '0'     1T mark, 1T space
 *   '1'     2T mark, 1T space
 *
 * So the SPACE never moves and the MARK carries the value, which is the same
 * bargain NEC makes and the exact opposite of the one EV1527 makes. The frame
 * repeats every 45 ms for as long as the key is held, and a remote sends it
 * at least three times per press whatever the key does.
 *
 * The bits go out LSB FIRST and the COMMAND is at the bottom, which matters
 * more here than it looks: a frame the record cut in half still says which
 * key was pressed. Above the command sits the device address, and the frame
 * length is what says how wide that field is:
 *
 *   12 bits   7 command, 5 address     the original, and still most of them
 *   15 bits   7 command, 8 address     the amplifiers and tuners
 *   20 bits   7 command, 5 address, 8 extended
 *
 * Nothing in the frame announces its length. What ends it is the gap: every
 * space inside a frame is 1T and the one after the last bit is the rest of
 * the 45 ms, so the decoder reads bits until a long gap and then asks whether
 * it stopped at 12, 15 or 20. A frame that stopped anywhere else is not one.
 *
 * WHAT IT MUST NOT BE CONFUSED WITH is RC6, the one other infrared leader in
 * this range - and the two are close enough that a decoder which only checked
 * "a long mark, then bits" would read one as the other. The difference is the
 * leader's own ratio: SIRC is 4:1, RC6 is 3:1 (2.666 ms of mark against 0.889
 * of space). T comes out of the MARK here, so an RC6 leader offers T = 666 us
 * and then its own space measures 1.33T, which the 1T window turns down. And
 * behind the leader the argument is settled several times over: RC6 is
 * bi-phase, so its spaces are half a bit or a whole one and vary with the
 * data, while every space in a SIRC frame is one unit. Twelve independent
 * checks of a constant a bi-phase code cannot hold.
 *
 * NEC is not a risk in either direction: its leader is 9 ms of mark, which
 * would put T at 2.25 ms, past the top of the window by a factor of two and a
 * half - and a SIRC leader read as NEC's is out by a factor of four.
 *
 * Pure C, host-testable.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "logic_decode.h"

/*- Definitions -------------------------------------------------------------*/
#define SIRC_LEAD_TICKS      4    // the leader's mark, in T - and where T comes from

// 600 us by the standard. The window is for the clones, and it is deliberately
// narrow at the top: 1000 us of T would put the leader at 4 ms, and nothing
// in this family sends one.
#define SIRC_T_MIN_NS   350000
#define SIRC_T_MAX_NS  1000000

// Below this a quarter of T is not resolvable and the 1T/2T split stops being
// a decision
#define SIRC_T_MIN_SAMPLES    3

// In tenths of T. The leader's space is held tightest of all, because it is
// the one field that separates this leader from RC6's - see the head of the
// file. The two mark windows must NOT meet: a mark of 1.4T is not a bit of
// either kind, and rounding it to the nearer one would read noise as data.
#define SIRC_LEAD_SP_MIN_X10   8
#define SIRC_LEAD_SP_MAX_X10  12
#define SIRC_SPACE_MIN_X10     7
#define SIRC_SPACE_MAX_X10    13
#define SIRC_MARK0_MIN_X10     7
#define SIRC_MARK0_MAX_X10    13
#define SIRC_MARK1_MIN_X10    15
#define SIRC_MARK1_MAX_X10    25

// A frame stands in silence: 45 ms between repeats, of which the frame itself
// is at most 33. Asked for only where the record shows the run before the
// leader whole - at the very start of the record that run was cut by the
// buffer, and its length is a fact about the buffer.
#define SIRC_QUIET_TICKS       5

#define SIRC_MAX_BITS         20
// The command is the bottom seven bits, so this is the point at which a frame
// the record cut short still says something true
#define SIRC_MIN_BITS          7

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_sirc  (g_logic_analysis.sirc)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
const SircAnalysis *sirc_analysis(void)
{
  return &g_sirc;
}

//-----------------------------------------------------------------------------
// Which frame byte `idx` belongs to, or NULL. A frame is two bytes or three
// and they are laid down in order, so a walk over the frames is exact.
static const SircFrame *sirc_frame_at(const SircAnalysis *a, int idx)
{
  if (idx < 0)
    return NULL;

  for (int i = 0; i < a->frames && i < SIRC_MAX_FRAMES; i++)
  {
    const SircFrame *f = &a->frame[i];

    if (idx >= f->first && idx < f->first + f->count)
      return f;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
// What the frame said, written once across the bytes it took. Decimal and not
// hex on purpose: Sony's own numbering is decimal - device 1 is a television,
// device 16 a video recorder - and every remote-code table ever printed lists
// them that way.
void sirc_byte_label(const SircAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  const SircFrame *f = sirc_frame_at(a, idx);

  if (!f)
    return;

  if (f->cut)
    snprintf(buf, size, "C%d cut", f->cmd);
  else if (20 == f->bits)
    snprintf(buf, size, "A%d C%d E%d", f->addr, f->cmd, f->ext);
  else
    snprintf(buf, size, "A%d C%d", f->addr, f->cmd);
}

//-----------------------------------------------------------------------------
// ...and which field a byte is on its own. The command comes FIRST off the
// wire, which is the opposite way round from how anyone writes a remote code
// down, so saying it beats leaving the reader to infer it from the order.
void sirc_field_label(const SircAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  (void)v;

  buf[0] = 0;

  const SircFrame *f = sirc_frame_at(a, idx);

  if (!f)
    return;

  static const char *const name[3] = { "CMD", "ADDR", "EXT" };

  snprintf(buf, size, "%s", name[idx - f->first]);
}

//-----------------------------------------------------------------------------
// One frame is one message - a key on a remote - so the address, command and
// extended byte light together and the reading is written once across them
void sirc_group_at(const SircAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  const SircFrame *f = sirc_frame_at(a, idx);

  if (!f)
    return;

  *start = f->first;
  *len = f->count;
}

//-----------------------------------------------------------------------------
// The sample span of bits [i..j] of a frame whose bit `n` began at run
// bit_run[n]. A bit ends where the next one starts; the last one has no
// successor and ends with its own mark, the space after it belonging to the
// gap in front of the next frame rather than to this bit.
static void sirc_span(const LogicScratch *s, const int *bit_run, int nbits,
    int i, int j, int *p0, int *p1)
{
  *p0 = s->pos[bit_run[i]];
  *p1 = (j + 1 < nbits) ? s->pos[bit_run[j + 1]] :
      s->pos[bit_run[j]] + s->len[bit_run[j]];
}

//-----------------------------------------------------------------------------
// One pass over the record. Returns the number of whole frames found; out and
// a may be NULL for a counting-only pass.
static int sirc_scan(const LogicScratch *s, int runs, int period_ns, int idle,
    LogicResult *out, SircAnalysis *a)
{
  int found = 0;

  for (int r = 1; r + 2 < runs; r++)
  {
    if ((0 != s->lvl[r]) == (0 != idle))
      continue;                        // the leader is a mark, not silence

    // T comes out of the leader's mark, which is four of them: an error in
    // the edge is divided by four before it reaches anything else
    int64_t t_x256 = (int64_t)s->len[r] * 256 / SIRC_LEAD_TICKS;

    if (t_x256 < SIRC_T_MIN_SAMPLES * 256)
      continue;

    int t_ns = (int)(t_x256 * period_ns / 256);

    if (t_ns < SIRC_T_MIN_NS || t_ns > SIRC_T_MAX_NS)
      continue;

    // ...and the leader's own space is 1T, which is the field that tells this
    // leader from RC6's. See the head of the file: RC6 offers 1.33T here.
    int64_t sp10 = (int64_t)s->len[r + 1] * 10 * 256 / t_x256;

    if (sp10 < SIRC_LEAD_SP_MIN_X10 || sp10 > SIRC_LEAD_SP_MAX_X10)
      continue;

    // Silence in front of it, or the record's own start
    if (r > 1 && (int64_t)s->len[r - 1] * 256 < t_x256 * SIRC_QUIET_TICKS)
      continue;

    uint32_t value = 0;
    int bit_run[SIRC_MAX_BITS];
    int n = 0, k = r + 2;
    bool bad = false, ended = false, ran_out = false;

    while (n < SIRC_MAX_BITS && k + 1 < runs && !ended && !bad && !ran_out)
    {
      int64_t m10 = (int64_t)s->len[k] * 10 * 256 / t_x256;
      int bit;

      if (m10 >= SIRC_MARK0_MIN_X10 && m10 <= SIRC_MARK0_MAX_X10)
        bit = 0;
      else if (m10 >= SIRC_MARK1_MIN_X10 && m10 <= SIRC_MARK1_MAX_X10)
        bit = 1;
      else
      {
        bad = true;
        break;
      }

      value |= (uint32_t)bit << n;      // LSB first: the command comes out first
      bit_run[n] = k;
      n++;

      // Every space inside a frame is one unit. Longer means the frame ended
      // and the gap to the next one has begun; shorter is a timing failure,
      // and those are different answers.
      int64_t g10 = (int64_t)s->len[k + 1] * 10 * 256 / t_x256;

      if (g10 > SIRC_SPACE_MAX_X10)
        ended = true;
      else if (g10 < SIRC_SPACE_MIN_X10)
      {
        // ...unless it is the record's own last run, which the buffer cut and
        // whose length is therefore a fact about the buffer. A short one
        // there says the record ran out, not that the timing failed.
        if (k + 1 == runs - 1)
          ran_out = true;
        else
          bad = true;
      }

      k += 2;
    }

    if (bad)
      continue;

    // Why the bit loop stopped decides what this frame is. A frame closed by
    // its own gap has to have stopped on one of the three lengths - nothing
    // else is a SIRC frame, and 13 bits is a signal that started like one and
    // then was not. Running out of RECORD is different: at 600 us a unit a
    // 20-bit frame is 33 ms, which is longer than the record holds at most
    // timebases, and the command is the BOTTOM seven bits - so a frame cut
    // short still says which key was pressed, and that is worth reporting
    // rather than refusing.
    bool cut = (!ended && (ran_out || k + 1 >= runs));
    bool whole = ended && (12 == n || 15 == n || 20 == n);

    if (!whole && !(cut && n >= SIRC_MIN_BITS))
      continue;

    found++;

    // Past the frame. k stands on the run AFTER the closing gap, which is the
    // next frame's leader, so the step back leaves the loop's own r++ landing
    // on it rather than stepping over it.
    if (!out || !a)
    {
      r = k - 1;
      continue;
    }

    int nbytes = cut ? 1 : ((20 == n) ? 3 : 2);

    if (out->count + nbytes > LOGIC_MAX_BYTES || a->frames >= SIRC_MAX_FRAMES)
    {
      out->truncated = true;
      break;
    }

    SircFrame *f = &a->frame[a->frames];

    f->bits = (uint8_t)n;
    f->cmd = (uint8_t)(value & 0x7F);
    f->addr = cut ? 0 :
        (uint8_t)((value >> 7) & ((15 == n) ? 0xFF : 0x1F));
    f->ext = (20 == n) ? (uint8_t)((value >> 12) & 0xFF) : 0;
    f->cut = cut;
    f->first = (uint8_t)out->count;
    f->count = (uint8_t)nbytes;

    // Which bits each byte came from, so a byte lights the part of the
    // waveform it was actually read out of rather than an even slice of the
    // frame - the bits are not equal in time and never were
    static const uint8_t lo[3] = { 0, 7, 12 };
    const uint8_t hi[3] = { 6, (uint8_t)((15 == n) ? 14 : 11), 19 };
    const uint8_t val[3] = { f->cmd, f->addr, f->ext };

    for (int i = 0; i < nbytes; i++)
    {
      int p0, p1;

      sirc_span(s, bit_run, n, lo[i], hi[i], &p0, &p1);

      out->bytes[out->count] = val[i];
      out->pos[out->count] = p0;
      out->end[out->count] = p1;
      out->count++;
    }

    a->frames++;
    a->t_ns = t_ns;

    out->overrun = out->overrun || cut;

    r = k - 1;
  }

  return found;
}

//-----------------------------------------------------------------------------
int sirc_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;

  memset(out, 0, sizeof(*out));
  out->proto = PROTO_SIRC;

  runs = logic_runs(data, size, offset, scratch, &mid);

  // A leader, its space and two runs per bit, with the record opening
  // somewhere in front. Bounded by the SHORTEST thing this decoder reports
  // and not by the shortest frame: a record too small for twelve bits is
  // exactly the record a cut frame comes out of, and asking for a whole one
  // here would turn away every frame the cut path exists to report.
  if (runs < 2 * SIRC_MIN_BITS + 3 || period_ns <= 0)
    return 0;

  // A demodulator rests with no carrier and the record ends between frames,
  // so the last run IS the idle level. Probe the LED driver instead of the
  // receiver and everything inverts, including this - and the reading comes
  // out the same, which is the point of asking the record rather than
  // assuming.
  int idle = (0 != scratch->lvl[runs - 1]) ? 1 : 0;

  memset(&g_sirc, 0, sizeof(g_sirc));

  if (0 == sirc_scan(scratch, runs, period_ns, idle, out, &g_sirc))
    return 0;

  if (0 == out->count)
    return 0;

  out->rate = g_sirc.t_ns;
  out->idle_high = (0 != idle);
  out->burst_start = true;           // a frame is found from its own leader

  // A key held down repeats the frame, and there is no checksum anywhere in
  // the protocol - so repeats agreeing is the only corroboration on offer. A
  // record long enough for several frames is long enough for a finger to have
  // moved, though, so disagreement is not an error: the header stops claiming
  // one reading and every frame keeps its own.
  g_sirc.agree = true;

  for (int i = 1; i < g_sirc.frames; i++)
  {
    const SircFrame *f = &g_sirc.frame[i], *f0 = &g_sirc.frame[0];

    if (f->cut || f0->cut || f->bits != f0->bits || f->cmd != f0->cmd ||
        f->addr != f0->addr || f->ext != f0->ext)
      g_sirc.agree = false;
  }

  const SircFrame *f = &g_sirc.frame[0];

  if (f->cut)
    snprintf(out->info, sizeof(out->info), "SIRC C=%d %d bits cut",
        f->cmd, f->bits);
  else if (!g_sirc.agree)
    snprintf(out->info, sizeof(out->info), "SIRC %d frames", g_sirc.frames);
  else
  {
    char rep[8] = "";

    if (g_sirc.frames > 1)
      snprintf(rep, sizeof(rep), " x%d", g_sirc.frames);

    if (20 == f->bits)
      snprintf(out->info, sizeof(out->info), "SIRC20 A=%d C=%d E=%d%s",
          f->addr, f->cmd, f->ext, rep);
    else
      snprintf(out->info, sizeof(out->info), "SIRC%d A=%d C=%d%s",
          f->bits, f->addr, f->cmd, rep);
  }

  return out->count;
}
