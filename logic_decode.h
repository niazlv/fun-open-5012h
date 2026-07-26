/*
 * Single-wire protocol decoders: shared result/scratch types and the
 * auto-detecting dispatcher. Pure C, host-testable.
 *
 * Every decoder consumes the same ring buffer of 8-bit samples and fills a
 * LogicResult, including the sample position of every decoded byte so the
 * UI can highlight bytes on the waveform and jump between them.
 */

#ifndef _LOGIC_DECODE_H_
#define _LOGIC_DECODE_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
#define LOGIC_MAX_BYTES  32
#define LOGIC_MAX_RUNS   512

/*- Types -------------------------------------------------------------------*/
typedef enum
{
  PROTO_NONE = 0,
  PROTO_AUTO = 0, // as a config value: try everything
  PROTO_UART = 1,
  PROTO_ONEWIRE,
  PROTO_WS2812,
  PROTO_NEC,
  PROTO_RAW,      // fallback: any digital signal, raw bits at the detected rate
  PROTO_COUNT,
} proto_t;

typedef struct
{
  proto_t  proto;
  int      rate;       // UART: baud; others: bit/slot time in ns
  bool     idle_high;
  int      count;      // decoded bytes
  int      errors;     // framing/timing errors
  char     info[32];   // one-line summary for the panel header
  uint8_t  bytes[LOGIC_MAX_BYTES];
  int      pos[LOGIC_MAX_BYTES];  // byte start, time-order sample index
  int      end[LOGIC_MAX_BYTES];  // byte end (exclusive), same units
} LogicResult;

// Working memory (~4.5 KB) provided by the caller: the firmware puts it in
// spare main SRAM, host tests use a static instance
typedef struct
{
  int      len[LOGIC_MAX_RUNS];
  int      pos[LOGIC_MAX_RUNS];
  uint8_t  lvl[LOGIC_MAX_RUNS];

  // Run-split cache. Every decoder in the auto cascade thresholds the SAME
  // record, and that is two full passes over 24K samples each time; the
  // dispatcher invalidates this once per call and the cascade then pays for
  // the split once instead of five times.
  const uint8_t *cache_data;
  int      cache_size;
  int      cache_offset;
  int      cache_runs;
  int      cache_mid;
  uint32_t cache_hash;
  uint32_t cache_valid;
} LogicScratch;

// Written into cache_valid by logic_runs(); the scratch lives in otherwise
// uninitialised SRAM, so the marker has to be something garbage will not be
#define LOGIC_CACHE_MAGIC  0x5ab1e10cu

/*- Prototypes --------------------------------------------------------------*/
// Individual decoders; each returns the number of bytes decoded (0 = the
// record does not look like this protocol)
int uart_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
int onewire_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
int ws2812_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);
int nec_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out);

// Dispatcher: forced = PROTO_AUTO tries every decoder and keeps the best
// fit (most bytes, then fewest errors); a specific proto runs only that one
int logic_decode(const uint8_t *data, int size, int offset, int period_ns,
    proto_t forced, LogicScratch *scratch, LogicResult *out);

// Shared helper: threshold the record into level runs with hysteresis.
// Returns the run count, 0 when the signal has no digital-looking swing.
// mid_out receives the threshold used.
int logic_runs(const uint8_t *data, int size, int offset,
    LogicScratch *scratch, int *mid_out);

const char *logic_proto_name(proto_t proto);

#endif // _LOGIC_DECODE_H_
