/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 1-Wire (Dallas/Maxim) decoder, standard speed, plus the transaction-level
 * reading that says WHICH device answered.
 *
 * Bus idles high. A reset is a >=400 us low pulse; the slave answers with a
 * presence pulse. Every bit slot starts with a falling edge: a short low
 * (1..15 us) is a '1', a long low (60..120 us) is a '0'; bits are LSB-first.
 *
 * Every 1-Wire device on earth speaks those slots identically, so the bytes
 * alone never say whether the thing on the probe is a thermometer or the key
 * from a door intercom. What says it is the transaction: a reset, then a ROM
 * command, then - depending on which - a 64-bit ROM whose first byte is the
 * family code and whose last byte is a CRC over the other seven, or a function
 * command with the device's own payload behind it. Read that structure and
 * DS18B20 (family 0x28, a nine-byte scratchpad with a temperature in it) and
 * DS1990A (family 0x01, a serial number and no function commands at all) stop
 * looking alike.
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
#define OW_RESET_MIN_NS     400000  // reset low pulse
#define OW_ONE_MAX_NS        20000  // low <= this = '1'
#define OW_ZERO_MIN_NS       45000  // low >= this = '0'
#define OW_ZERO_MAX_NS      150000  // longer lows (but < reset) are errors

/*- Types -------------------------------------------------------------------*/
// Where in a transaction the byte walker is. A record is a window onto a bus
// that was already talking, so it starts in UNKNOWN and only a reset pulse
// puts it on solid ground.
enum
{
  EXP_UNKNOWN = 0,
  EXP_ROMCMD,
  EXP_ROM,
  EXP_FNCMD,
  EXP_SP,
  EXP_WRSP,
  EXP_SEARCH,
  EXP_PIO,       // a DS2413's answer to PIO ACCESS READ
};

/*- Variables ---------------------------------------------------------------*/
// Shared with every other decoder: only one analysis is live at a time, and
// the cascade guarantees this one is it. See LogicAnalysis in logic_decode.h.
#define g_ow    (g_logic_analysis.ow)

// The ROM layer: eight commands that every device on every bus speaks
// identically, whatever it is. These are the only 1-Wire opcodes that mean
// one thing everywhere.
static const struct { uint8_t v; const char *name; } g_ow_cmd[] =
{
  { 0x33, "READ ROM" }, { 0x0F, "READ ROM" }, { 0x55, "MATCH" },
  { 0xCC, "SKIP" },     { 0xF0, "SEARCH" },   { 0xEC, "ALARM" },
  { 0x69, "OD MATCH" }, { 0x3C, "OD SKIP" },
};

// ...and the FUNCTION layer, which does not. Above the ROM every family
// invents its own opcodes, and they collide: 0xF5 reads eight channels on a
// DS2408 and two pins on a DS2413, 0xF0 reads memory on an EEPROM and the
// PIO registers on a switch, 0xAA is READ SCRATCHPAD on one part and READ
// STATUS on another. A single table of "1-Wire commands" therefore cannot
// exist, and one that pretends to will name half of them wrongly.
//
// So the family code decides. It came past in the ROM, eight bytes with a
// CRC over them, which is the one thing on this bus that says WHAT is
// answering - and until it has, a function byte stays a number.
typedef enum
{
  OWC_UNKNOWN = 0,
  OWC_KEY,       // a serial number and nothing else: iButtons, DS2401
  OWC_THERM,     // DS18x20 and the parts that share its scratchpad
  OWC_NVRAM,     // the memory iButtons: scratchpad, then copy
  OWC_EEPROM,    // DS2431/2433/28E04: the same shape, different sizes
  OWC_EPROM,     // add-only memory: DS1982, DS2502/5/6
  OWC_SWITCH,    // DS2408, eight channels
  OWC_SWITCH2,   // DS2413, two - the one inside a MagSafe connector
  OWC_SWITCH1,   // DS2405/2406/2407
  OWC_ADC,       // DS2450
  OWC_BATT,      // DS2438, DS2760
  OWC_COUNTER,   // DS2423
  OWC_RTC,
} ow_class_t;

static const struct { uint8_t cls; uint8_t v; const char *name; } g_ow_fn[] =
{
  { OWC_THERM,   0x44, "CONVERT" },   { OWC_THERM,   0xBE, "READ SP" },
  { OWC_THERM,   0x4E, "WRITE SP" },  { OWC_THERM,   0x48, "COPY SP" },
  { OWC_THERM,   0xB8, "RECALL" },    { OWC_THERM,   0xB4, "READ PWR" },

  { OWC_SWITCH,  0xF0, "READ PIO" },  { OWC_SWITCH,  0xF5, "CH READ" },
  { OWC_SWITCH,  0x5A, "CH WRITE" },  { OWC_SWITCH,  0xC3, "RST LATCH" },
  { OWC_SWITCH,  0xCC, "WR COND" },

  { OWC_SWITCH2, 0xF5, "PIO READ" },  { OWC_SWITCH2, 0x5A, "PIO WRITE" },

  { OWC_SWITCH1, 0xF0, "READ MEM" },  { OWC_SWITCH1, 0xF5, "CH ACCESS" },
  { OWC_SWITCH1, 0xAA, "READ ST" },   { OWC_SWITCH1, 0x55, "WRITE ST" },

  { OWC_EEPROM,  0x0F, "WRITE SP" },  { OWC_EEPROM,  0xAA, "READ SP" },
  { OWC_EEPROM,  0x55, "COPY SP" },   { OWC_EEPROM,  0xF0, "READ MEM" },

  { OWC_NVRAM,   0x0F, "WRITE SP" },  { OWC_NVRAM,   0xAA, "READ SP" },
  { OWC_NVRAM,   0x55, "COPY SP" },   { OWC_NVRAM,   0xF0, "READ MEM" },

  { OWC_EPROM,   0xF0, "READ MEM" },  { OWC_EPROM,   0xC3, "READ ST" },
  { OWC_EPROM,   0xA5, "READ CRC" },  { OWC_EPROM,   0x0F, "WRITE MEM" },
  { OWC_EPROM,   0x55, "WRITE ST" },

  { OWC_ADC,     0xAA, "READ MEM" },  { OWC_ADC,     0x55, "WRITE MEM" },
  { OWC_ADC,     0x3C, "CONVERT" },

  { OWC_BATT,    0x4E, "WRITE SP" },  { OWC_BATT,    0xBE, "READ SP" },
  { OWC_BATT,    0x48, "COPY SP" },   { OWC_BATT,    0xB8, "RECALL" },
  { OWC_BATT,    0x44, "CONVERT T" }, { OWC_BATT,    0xB4, "CONVERT V" },

  { OWC_COUNTER, 0xA5, "READ CNT" },  { OWC_COUNTER, 0xF0, "READ MEM" },
  { OWC_COUNTER, 0x0F, "WRITE SP" },  { OWC_COUNTER, 0xAA, "READ SP" },
  { OWC_COUNTER, 0x5A, "COPY SP" },
};

// Family codes: the first ROM byte, and the whole point of reading a ROM.
// Names are short because they end up in a 32-character panel header next to
// a temperature.
static const struct { uint8_t code; uint8_t cls; const char *name; }
    g_ow_family[] =
{
  { 0x01, OWC_KEY,     "DS1990 key" },  // and DS2401/DS2411: a serial number
  { 0x02, OWC_NVRAM,   "DS1991 key" },  // MultiKey, three secure subkeys
  { 0x04, OWC_RTC,     "DS1994 RTC" },  // and DS2404: 4Kb NVRAM and a clock
  { 0x05, OWC_SWITCH1, "DS2405 sw" },
  { 0x06, OWC_NVRAM,   "DS1993 mem" },  // 4Kb
  { 0x08, OWC_NVRAM,   "DS1992 mem" },  // 1Kb
  { 0x09, OWC_EPROM,   "DS1982 mem" },  // and DS2502: 1Kb, write once
  { 0x0A, OWC_NVRAM,   "DS1995 mem" },  // 16Kb
  { 0x0B, OWC_EPROM,   "DS2505 eep" },  // and DS1985: 16Kb
  { 0x0C, OWC_NVRAM,   "DS1996 mem" },  // 64Kb
  { 0x0F, OWC_EPROM,   "DS2506 eep" },  // and DS1986: 64Kb
  { 0x10, OWC_THERM,   "DS18S20" },     // and DS1920: half-degree steps
  { 0x12, OWC_SWITCH1, "DS2406 sw" },   // and DS2407
  { 0x14, OWC_EEPROM,  "DS2430 eep" },  // and DS1971: 256 bits
  { 0x16, OWC_NVRAM,   "DS1954 java" }, // the cryptographic Java iButton
  { 0x18, OWC_NVRAM,   "DS1963S" },     // the monetary iButton, SHA-1
  { 0x19, OWC_UNKNOWN, "DS28E17 i2c" }, // a 1-Wire to I2C bridge
  { 0x1A, OWC_NVRAM,   "DS1963L" },
  { 0x1B, OWC_COUNTER, "DS2436 batt" },
  { 0x1C, OWC_EEPROM,  "DS28E04 eep" }, // 4Kb and two PIO
  { 0x1D, OWC_COUNTER, "DS2423 cnt" },  // 4Kb NVRAM and four counters
  { 0x1E, OWC_BATT,    "DS2437 batt" },
  { 0x1F, OWC_SWITCH1, "DS2409 hub" },  // the coupler that branches a net
  { 0x20, OWC_ADC,     "DS2450 adc" },  // four channels
  { 0x21, OWC_THERM,   "DS1921 log" },  // Thermochron
  { 0x22, OWC_THERM,   "DS1822" },
  { 0x23, OWC_EEPROM,  "DS2433 eep" },  // and DS1973: 4Kb
  { 0x24, OWC_RTC,     "DS1904 RTC" },  // and DS2415
  { 0x26, OWC_BATT,    "DS2438 batt" },
  { 0x27, OWC_RTC,     "DS2417 RTC" },
  { 0x28, OWC_THERM,   "DS18B20" },     // and MAX31820
  { 0x29, OWC_SWITCH,  "DS2408 sw" },   // eight channels
  { 0x2C, OWC_ADC,     "DS2890 pot" },
  { 0x2D, OWC_EEPROM,  "DS2431 eep" },  // and DS1972: 1Kb
  { 0x2E, OWC_BATT,    "DS2770 batt" },
  { 0x30, OWC_BATT,    "DS2760 batt" }, // and DS2761/DS2762
  { 0x31, OWC_BATT,    "DS2720 batt" },
  { 0x32, OWC_BATT,    "DS2780 batt" },
  { 0x33, OWC_EEPROM,  "DS1961S key" }, // and DS2432: 1Kb with SHA-1
  { 0x34, OWC_BATT,    "DS2703 auth" },
  { 0x35, OWC_BATT,    "DS2755 batt" },
  { 0x36, OWC_BATT,    "DS2740 batt" },
  { 0x37, OWC_EPROM,   "DS1977 mem" },  // 32Kb behind a password
  { 0x3A, OWC_SWITCH2, "DS2413 sw" },   // ...and what a MagSafe plug has
  { 0x3B, OWC_THERM,   "DS1825" },      // MAX31826 and MAX31850 share it
  { 0x41, OWC_THERM,   "DS1922 log" },  // and DS1923/DS2422
  { 0x42, OWC_THERM,   "DS28EA00" },    // temperature with PIO and chain
  { 0x43, OWC_EEPROM,  "DS28EC20 eep" },// 20Kb
  { 0x44, OWC_EEPROM,  "DS28E10 auth" },
  { 0x51, OWC_BATT,    "DS2751 batt" },
  // The high half is the same silicon in other packages, plus one part that
  // turns up on its own: 0x81 is a DS1420, a ROM and nothing else -
  // functionally a DS1990 sold as an identity token rather than as a key,
  // and what a DS9097U adapter has inside it.
  { 0x81, OWC_KEY,     "DS1420 ID" },
  { 0x84, OWC_RTC,     "DS2404S RTC" },
  { 0x89, OWC_EPROM,   "DS2502-E48" },  // an EPROM holding a MAC address
  { 0x8B, OWC_EPROM,   "DS1985U mem" },
  { 0x8F, OWC_EPROM,   "DS1986U mem" },
};


// Scratchpad layout, DS18B20: temperature, the two alarm bytes, the
// configuration register, three reserved bytes and a CRC over all eight.
// The reserved ones are not noise - a working part returns FF, something,
// 10 - so they are labelled, and a part that returns anything else there is
// saying something about itself.
//
// A DS18S20 uses the same nine bytes differently: byte 4 is reserved rather
// than a configuration register, and bytes 6 and 7 are the COUNT REMAIN and
// COUNT PER C that give it a reading finer than its half-degree step. Which
// of the two layouts is on the wire is decided once, and the labels follow.
static const uint8_t g_ow_sp_role[9] =
{
  OW_R_TEMPL, OW_R_TEMPH, OW_R_TH, OW_R_TL, OW_R_CFG,
  OW_R_RSV_FF, OW_R_RSVD, OW_R_RSV_10, OW_R_SPCRC,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Dallas CRC8, x^8 + x^5 + x^4 + 1 fed LSB-first. Over a whole ROM or a whole
// scratchpad (CRC byte included) a good one comes out zero.
static uint8_t ow_crc8(const uint8_t *d, int n)
{
  uint8_t crc = 0;

  while (n--)
  {
    uint8_t b = *d++;

    for (int i = 0; i < 8; i++)
    {
      uint8_t mix = (uint8_t)((crc ^ b) & 1);

      crc >>= 1;

      if (mix)
        crc ^= 0x8C;

      b >>= 1;
    }
  }

  return crc;
}

//-----------------------------------------------------------------------------
static const char *ow_cmd_name(uint8_t v)
{
  for (unsigned i = 0; i < sizeof(g_ow_cmd) / sizeof(g_ow_cmd[0]); i++)
  {
    if (g_ow_cmd[i].v == v)
      return g_ow_cmd[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
static const char *ow_family_name(uint8_t code)
{
  for (unsigned i = 0; i < sizeof(g_ow_family) / sizeof(g_ow_family[0]); i++)
  {
    if (g_ow_family[i].code == code)
      return g_ow_family[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
static uint8_t ow_family_class(uint8_t code)
{
  for (unsigned i = 0; i < sizeof(g_ow_family) / sizeof(g_ow_family[0]); i++)
  {
    if (g_ow_family[i].code == code)
      return g_ow_family[i].cls;
  }

  return OWC_UNKNOWN;
}

//-----------------------------------------------------------------------------
static bool ow_family_is_thermometer(uint8_t code)
{
  return (OWC_THERM == ow_family_class(code));
}

//-----------------------------------------------------------------------------
// A function byte, named by the family that answered. With no ROM on the wire
// there is no family, and the DS18x20 set is the only reading worth offering
// - it is what a 1-Wire bus is nearly always for - but it is offered as a
// guess and not as a fact, which is what the trailing '?' says.
static const char *ow_fn_name(uint8_t v, uint8_t family, bool known,
    bool *sure)
{
  uint8_t cls = known ? ow_family_class(family) : OWC_THERM;

  *sure = known;

  // A part this decoder can NAME but whose opcodes it does not have - a
  // DS28E17 bridge, say - gets nothing rather than a guess. Offering the
  // DS18x20 set there would be worse than saying nothing, because the family
  // code has already ruled it out.
  if (known && OWC_UNKNOWN == cls)
    return NULL;

  for (unsigned i = 0; i < sizeof(g_ow_fn) / sizeof(g_ow_fn[0]); i++)
  {
    if (g_ow_fn[i].cls == cls && g_ow_fn[i].v == v)
      return g_ow_fn[i].name;
  }

  return NULL;
}

//-----------------------------------------------------------------------------
// SEARCH ROM does not put the ROM on the wire as bytes: every ROM bit costs
// three slots - the device sends the bit, then its complement, then the master
// writes back the branch it takes. That third slot is the bit of the device
// that stays selected, so the ROM is in there, just at one third scale. The
// byte packing is only a container and the slot order survives it, so the
// triplets can be picked back out of the decoded bytes.
static bool ow_search_rom(const LogicResult *r, int from, uint8_t *rom)
{
  int need = (3 * 64 + 7) / 8;  // 192 slots

  if (from < 0 || r->count - from < need)
    return false;

  memset(rom, 0, 8);

  for (int j = 0; j < 64; j++)
  {
    int k = 3 * j + 2;
    int bit = (r->bytes[from + k / 8] >> (k % 8)) & 1;

    rom[j / 8] |= (uint8_t)(bit << (j % 8));
  }

  // Nothing but the CRC says these 64 bits were a ROM rather than three
  // devices arbitrating, so an unchecked search result is not reported
  return (0 == ow_crc8(rom, 8));
}

//-----------------------------------------------------------------------------
// Walk the decoded bytes as transactions and work out what is on the bus.
// `at_reset[i]` marks the bytes that came first after a reset pulse: that is
// the one place in the stream where the meaning of a byte is known outright.
static void ow_analyze(const LogicResult *r, const bool *at_reset, OwAnalysis *a)
{
  int st = EXP_UNKNOWN;
  int field = 0;
  int search_at = -1;
  uint8_t sp[9];
  int sp_len = 0;
  bool sp_crc_ok = false;
  bool sp_done = false;      // a checked scratchpad is not lost to a later partial one
  bool saw_temp_cmd = false;
  uint8_t cfg = 0;
  bool have_cfg = false;

  memset(a, 0, sizeof(*a));
  memset(sp, 0, sizeof(sp));

  for (int i = 0; i < r->count; i++)
  {
    uint8_t v = r->bytes[i];

    if (at_reset[i])
    {
      st = EXP_ROMCMD;
      field = 0;
    }

    switch (st)
    {
      case EXP_ROMCMD:
        a->role[i] = OW_R_ROMCMD;

        switch (v)
        {
          case 0x33: case 0x0F:            // READ ROM: the device answers
          case 0x55: case 0x69:            // MATCH ROM: the master addresses
            st = EXP_ROM;
            field = 0;
            break;

          case 0xCC: case 0x3C:            // SKIP ROM: no ROM, straight to work
            st = EXP_FNCMD;
            break;

          case 0xF0: case 0xEC:            // SEARCH ROM: triplets from here on
            st = EXP_SEARCH;
            search_at = i + 1;
            break;

          default:                          // not a ROM command: framing is off
            a->role[i] = OW_R_NONE;         // or this is a device with its own
            st = EXP_UNKNOWN;               // ideas. Either way, stop guessing.
            break;
        }
        break;

      case EXP_ROM:
        a->rom[field] = v;
        a->role[i] = (0 == field) ? OW_R_FAMILY :
            (field < 7) ? OW_R_SERIAL : OW_R_ROMCRC;

        if (++field == 8)
        {
          a->rom_seen = true;
          a->rom_valid = (0 == ow_crc8(a->rom, 8));
          st = EXP_FNCMD;
        }
        break;

      case EXP_FNCMD:
        a->role[i] = OW_R_FNCMD;

        switch (v)
        {
          case 0xBE:                        // READ SCRATCHPAD
            st = EXP_SP;
            field = 0;
            saw_temp_cmd = true;
            if (!sp_done)
              sp_len = 0;
            break;

          case 0x4E:                        // WRITE SCRATCHPAD: TH, TL, config
            st = EXP_WRSP;
            field = 0;
            saw_temp_cmd = true;
            break;

          case 0x44:                        // CONVERT T
            saw_temp_cmd = true;
            st = EXP_UNKNOWN;
            break;

          case 0x48: case 0xB8: case 0xB4:  // COPY SP / RECALL / READ POWER
            st = EXP_UNKNOWN;
            break;

          case 0xF5:
            // PIO ACCESS READ - but only on the part that has it. 0xF5 is
            // CHANNEL ACCESS READ on a DS2408 and a memory command on a
            // DS2406, and reading two pins out of any of those would be
            // inventing them. The ROM has to have said DS2413 first.
            if (a->rom_valid && 0x3A == a->rom[0])
            {
              st = EXP_PIO;
              field = 0;
            }
            else
            {
              st = EXP_UNKNOWN;
            }
            break;

          default:
            a->role[i] = OW_R_NONE;
            st = EXP_UNKNOWN;
            break;
        }
        break;

      case EXP_SP:
        a->role[i] = g_ow_sp_role[field];

        if (!sp_done && field < 9)
        {
          sp[field] = v;

          if (field + 1 > sp_len)
            sp_len = field + 1;
        }

        if (++field == 9)
        {
          if (!sp_done)
          {
            sp_crc_ok = (0 == ow_crc8(sp, 9));
            sp_done = sp_crc_ok;
          }

          st = EXP_UNKNOWN;
        }
        break;

      case EXP_WRSP:
        // The master writing the alarm registers and the resolution. Same
        // three bytes as the middle of the scratchpad, which is where the
        // configuration byte comes from when nothing was ever read back.
        a->role[i] = (0 == field) ? OW_R_TH : (1 == field) ? OW_R_TL : OW_R_CFG;

        if (2 == field)
        {
          cfg = v;
          have_cfg = true;
        }

        if (++field == 3)
          st = EXP_UNKNOWN;
        break;

      case EXP_PIO:
        // One byte: the two pins, their two output latches, and the same
        // four inverted in the top nibble. That complement is what says the
        // byte is a PIO answer and not the bus being read at the wrong
        // moment - a device that has let go reads 0xFF, which fails it.
        if (0 == field && ((v ^ 0xF0) >> 4) == (v & 0x0F))
        {
          a->pio_valid = true;
          a->pio = v;
          a->role[i] = OW_R_PIO;
        }
        else
        {
          a->role[i] = OW_R_NONE;
        }

        field++;
        st = EXP_UNKNOWN;
        break;

      case EXP_SEARCH:
        // One label for the whole triplet run: 24 bytes of "SRCH" under the
        // trace says nothing the first one has not already said
        a->role[i] = (i == search_at) ? OW_R_SEARCH : OW_R_NONE;
        break;

      default:
        a->role[i] = OW_R_NONE;
        break;
    }
  }

  if (!a->rom_valid && search_at >= 0)
  {
    uint8_t rom[8];

    if (ow_search_rom(r, search_at, rom))
    {
      memcpy(a->rom, rom, 8);
      a->rom_seen = true;
      a->rom_valid = true;
    }
  }

  // The ROM, when its CRC checks out, is the last word on what is on the bus
  if (a->rom_valid)
  {
    a->family = a->rom[0];
    a->device = ow_family_name(a->family);
    a->sure = true;
    a->thermometer = ow_family_is_thermometer(a->family);
  }

  // A key TOUCHED to a reader. The whole conversation is a reset, READ ROM
  // and eight bytes - a DS1990 has no function commands to send, so a record
  // that holds one and nothing after it is not a truncated exchange, it is
  // the complete one. Naming the event and not just the part is the point:
  // "DS1990 key" says what is on the bus and "iButton 0000123456" says what
  // just happened and which key did it.
  if (a->rom_valid)
  {
    bool any_fn = false;

    for (int k = 0; k < LOGIC_MAX_BYTES; k++)
    {
      if (OW_R_FNCMD == a->role[k])
        any_fn = true;
    }

    a->ibutton = !any_fn && (OWC_KEY == ow_family_class(a->family));
    // Only where there is no NAME to show. A DS18B20 whose record caught
    // just the ROM read is better reported as a DS18B20 than as twelve hex
    // digits - the name is the information there, and the serial is not.
    a->id_only = !any_fn && !a->ibutton && (NULL == a->device);
  }

  if (9 == sp_len && sp_crc_ok)
  {
    a->sp_valid = true;
    cfg = sp[4];
    have_cfg = true;
  }

  // Which of the two layouts the nine bytes are in. The DS18B20 line counts
  // in sixteenths and keeps a configuration register in byte 4, where the
  // undefined bits read as ones - 1F, 3F, 5F or 7F and nothing else. The
  // DS18S20 counts in halves and leaves that byte at FF. A byte that is
  // neither belongs to a part this decoder cannot name, and guessing DS18S20
  // out of "not a configuration register" would be exactly that.
  enum { PART_UNKNOWN, PART_B20, PART_S20 } part = PART_UNKNOWN;

  if (a->family)
    part = (0x10 == a->family) ? PART_S20 :
        ow_family_is_thermometer(a->family) ? PART_B20 : PART_UNKNOWN;
  else if (have_cfg)
    part = ((cfg & 0x9F) == 0x1F) ? PART_B20 :
        (0xFF == cfg) ? PART_S20 : PART_UNKNOWN;

  a->s20 = (PART_S20 == part);

  if (sp_len >= 2)
  {
    uint16_t u = (uint16_t)(((uint16_t)sp[1] << 8) | sp[0]);
    int mc;

    if (PART_S20 == part)
    {
      int raw = (int)(int16_t)u;
      int cpc = sp[7], cr = sp[6];

      // Half a degree is the step, but not the resolution: the part also
      // reports how far through the count it was when the conversion
      // stopped, and the datasheet's own arithmetic turns that into
      // sixteenths. Both bytes are the ones a DS18B20 leaves reserved.
      if (sp_len >= 9 && sp_crc_ok && cpc > 0 && cr <= cpc)
      {
        mc = (raw >> 1) * 1000 - 250 + (cpc - cr) * 1000 / cpc;
        a->ext_res = true;
      }
      else
        mc = raw * 500;
    }
    else
    {
      // Below 12 bits the low bits are undefined rather than zero
      a->bits = have_cfg ? 9 + ((cfg >> 5) & 3) : 12;
      u &= (uint16_t)~((1u << (12 - a->bits)) - 1u);
      mc = (int)(int16_t)u * 125 / 2;
    }

    // A checked scratchpad needs no second opinion; two bytes read out of an
    // aborted one are believed only if they land in the range a DS18x20 can
    // actually report
    if (sp_crc_ok || (mc >= -55000 && mc <= 125000))
    {
      a->temp_mc = mc;
      a->temp_valid = true;
      a->thermometer = true;
    }
    else
    {
      a->bits = 0;
      a->ext_res = false;
    }
  }

  if (a->sp_valid)
  {
    a->thermometer = true;

    // SKIP ROM means the family code was never sent - and on a bus with one
    // device that is how nearly every library talks to it. The scratchpad
    // still settles which line the part is from, so it gets named, with the
    // '?' that says the name came from its behaviour and not from its ROM:
    // DS1822 and DS1825 answer byte for byte like a DS18B20.
    if (!a->device && PART_UNKNOWN != part)
      a->device = (PART_S20 == part) ? "DS18S20" : "DS18B20";
    else if (!a->device)
      a->device = "DS18x20";   // the class is proven, the part is not
  }
  else if (!a->device && saw_temp_cmd)
  {
    a->device = "DS18x20";
    a->thermometer = true;
  }
}

//-----------------------------------------------------------------------------
const OwAnalysis *onewire_analysis(void)
{
  return &g_ow;
}

//-----------------------------------------------------------------------------
// What byte `idx` is, for the row under the number on the trace and on the
// panel. Empty when there is nothing to say: the caller fills the gap.
void onewire_byte_label(const OwAnalysis *a, int idx, uint8_t v,
    char *buf, int size)
{
  const char *name;

  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  switch (a->role[idx])
  {
    case OW_R_ROMCMD:
      name = ow_cmd_name(v);

      if (name)
        snprintf(buf, size, "%s", name);
      break;

    case OW_R_FNCMD:
    {
      // Above the ROM every family has its own opcodes and they collide, so
      // this byte is named by whatever the ROM said was answering. Without a
      // ROM the name is a guess and carries a '?' to say so.
      bool sure = false;

      name = ow_fn_name(v, a->family, a->rom_valid, &sure);

      if (name)
        snprintf(buf, size, "%s%s", name, sure ? "" : "?");
      break;
    }

    case OW_R_FAMILY:
      // The device name goes where the byte that says it is, and a question
      // mark where the ROM's own CRC did not agree
      name = ow_family_name(v);

      if (name)
        snprintf(buf, size, "%s%s", name, a->rom_valid ? "" : "?");
      else
        snprintf(buf, size, "fam%02X", v);
      break;

    case OW_R_PIO:
      // What the two pins are actually AT, which is the whole reason anyone
      // reads a DS2413: bit 0 is PIOA's level and bit 2 is PIOB's
      snprintf(buf, size, "A=%d B=%d", (v >> 0) & 1, (v >> 2) & 1);
      break;

    case OW_R_SERIAL:
      snprintf(buf, size, "SN");
      break;

    case OW_R_ROMCRC:
      snprintf(buf, size, a->rom_valid ? "CRC" : "CRC!");
      break;

    case OW_R_SEARCH:
      snprintf(buf, size, "SEARCH");
      break;

    case OW_R_TEMPL:
      snprintf(buf, size, "TEMP");
      break;

    case OW_R_TEMPH:
      // The reading itself, at the byte that completes it
      if (a->temp_valid)
      {
        int t = a->temp_mc;
        const char *sign = (t < 0) ? "-" : "+";

        if (t < 0)
          t = -t;

        snprintf(buf, size, "%s%d.%02dC", sign, t / 1000, (t % 1000) / 10);
      }
      else
        snprintf(buf, size, "T-HI");
      break;

    case OW_R_TH:
      snprintf(buf, size, "TH");
      break;

    case OW_R_TL:
      snprintf(buf, size, "TL");
      break;

    case OW_R_CFG:
      if (a->s20)
        snprintf(buf, size, "RSV");   // no configuration register on this one
      else if (a->bits >= 9 && a->bits <= 12)
        snprintf(buf, size, "%dbit", a->bits);
      else
        snprintf(buf, size, "CFG");
      break;

    case OW_R_RSVD:
      // Byte 6: reserved on a DS18B20, the remainder of the conversion on a
      // DS18S20 - one of the two bytes its finer reading is worked out of
      snprintf(buf, size, a->s20 ? "CNTR" : "RSV");
      break;

    case OW_R_RSV_FF:
    case OW_R_RSV_10:
      // A reserved byte is still a byte the part is supposed to return a
      // known value in. The CRC has already said the read was clean, so one
      // that reads differently is the part itself differing - which is what
      // a counterfeit DS18B20 does.
      if (a->s20 && OW_R_RSV_10 == a->role[idx])
        snprintf(buf, size, "CNT/C");
      else
        snprintf(buf, size, (a->sp_valid &&
            v != (OW_R_RSV_FF == a->role[idx] ? 0xFF : 0x10)) ? "RSV!" : "RSV");
      break;

    case OW_R_SPCRC:
      snprintf(buf, size, a->sp_valid ? "CRC" : "CRC!");
      break;

    default:
      break;
  }
}

//-----------------------------------------------------------------------------
// Which byte of the scratchpad this is, rather than what the pair adds up to.
// Everything but the temperature already answers per byte.
void onewire_field_label(const OwAnalysis *a, int idx, uint8_t v, char *buf,
    int size)
{
  buf[0] = 0;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (OW_R_TEMPL == a->role[idx])
    snprintf(buf, size, "T-LO");
  else if (OW_R_TEMPH == a->role[idx])
    snprintf(buf, size, "T-HI");
  else
    onewire_byte_label(a, idx, v, buf, size);
}

//-----------------------------------------------------------------------------
// The temperature is two scratchpad bytes and one number. The ROM's eight
// are NOT grouped: the family code, the serial and the CRC are three
// different things worth saying, and a single "28FF641E0C279A" across all of
// them would say less than the labels do.
void onewire_group_at(const OwAnalysis *a, int idx, int *start, int *len)
{
  *start = idx;
  *len = 1;

  if (idx < 0 || idx >= LOGIC_MAX_BYTES)
    return;

  if (OW_R_TEMPL == a->role[idx])
    *len = 2;
  else if (OW_R_TEMPH == a->role[idx] && idx > 0)
  {
    *start = idx - 1;
    *len = 2;
  }
}

//-----------------------------------------------------------------------------
// The panel header: what answered, and what it said
static void ow_build_info(const OwAnalysis *a, bool presence, int errors,
    char *buf, int size)
{
  char temp[16] = "";

  if (a->temp_valid)
  {
    int t = a->temp_mc;
    const char *sign = (t < 0) ? "-" : "+";

    if (t < 0)
      t = -t;

    snprintf(temp, sizeof(temp), "%s%d.%02dC", sign, t / 1000, (t % 1000) / 10);
  }

  // A key touched to a reader: the identity is the whole message, so it goes
  // in the header the way a reader would print it - the six serial bytes,
  // most significant first, which is how an iButton's number is written on
  // the key itself
  if (a->ibutton)
  {
    snprintf(buf, size, "iButton %02X%02X%02X%02X%02X%02X",
        a->rom[6], a->rom[5], a->rom[4], a->rom[3], a->rom[2], a->rom[1]);

    return;
  }

  // A ROM read and nothing else, from a family this decoder has no name for.
  // The transaction is still exactly one thing - an identity was asked for
  // and given - so the identity is what to show. Saying "fam 81 ok" instead
  // reports how much the DECODER knows, when what was on the wire was a
  // complete and perfectly readable answer.
  if (a->id_only)
  {
    snprintf(buf, size, "1-Wire %02X %02X%02X%02X%02X%02X%02X", a->family,
        a->rom[6], a->rom[5], a->rom[4], a->rom[3], a->rom[2], a->rom[1]);

    return;
  }

  if (a->pio_valid && a->device)
  {
    snprintf(buf, size, "%s A=%d B=%d", a->device, (a->pio >> 0) & 1,
        (a->pio >> 2) & 1);

    return;
  }

  const char *q = a->sure ? "" : "?";
  char how[8] = "";

  // How the reading was arrived at: the conversion resolution the part was
  // configured for, or - on a DS18S20 - that its count registers were used
  // to get sixteenths out of a half-degree step
  if (a->bits >= 9 && a->bits <= 12)
    snprintf(how, sizeof(how), " %db", a->bits);
  else if (a->ext_res)
    snprintf(how, sizeof(how), " ext");

  if (a->device && a->temp_valid)
    snprintf(buf, size, "%s%s %s%s", a->device, q, temp, how);
  else if (a->device && a->rom_valid)
    snprintf(buf, size, "%s ROM ok", a->device);
  else if (a->device)
    snprintf(buf, size, "%s%s by cmds", a->device, q);
  else if (a->temp_valid)
    snprintf(buf, size, "1-Wire %s", temp);
  else if (a->rom_valid)
    snprintf(buf, size, "1-Wire fam %02X ok", a->family);
  else if (a->rom_seen)
    snprintf(buf, size, "1-Wire ROM CRC bad");
  else
    snprintf(buf, size, "1-Wire%s err %d", presence ? " +presence" : "", errors);
}

//-----------------------------------------------------------------------------
int onewire_decode(const uint8_t *data, int size, int offset, int period_ns,
    LogicScratch *scratch, LogicResult *out)
{
  int mid, runs;
  bool seen_reset = false;
  bool presence = false;
  // Which decoded bytes were the first after a reset pulse - the only bytes
  // in the stream whose meaning is not a guess
  bool at_reset[LOGIC_MAX_BYTES];

  (void)data;

  memset(out, 0, sizeof(*out));
  memset(at_reset, 0, sizeof(at_reset));
  out->proto = PROTO_ONEWIRE;
  out->idle_high = true;

  runs = logic_runs(data, size, offset, scratch, &mid);

  if (runs < 6)
    return 0;

  // The bus must idle high: the longest run has to be a high level
  int longest = 0;

  for (int r = 1; r < runs; r++)
  {
    if (scratch->len[r] > scratch->len[longest])
      longest = r;
  }

  if (0 == scratch->lvl[longest])
    return 0;

  int value = 0, nbits = 0, byte_start = -1;
  int short_lows = 0;

  for (int r = 0; r < runs && out->count < LOGIC_MAX_BYTES; r++)
  {
    if (scratch->lvl[r] != 0)
      continue; // only low pulses carry information

    int64_t low_ns = (int64_t)scratch->len[r] * period_ns;

    if (low_ns >= OW_RESET_MIN_NS)
    {
      // Reset: restart byte assembly; the next low is the presence pulse
      seen_reset = true;
      value = 0;
      nbits = 0;
      byte_start = -1;
      at_reset[out->count] = true;

      if (r + 2 < runs && scratch->lvl[r + 2] == 0)
      {
        int64_t pres_ns = (int64_t)scratch->len[r + 2] * period_ns;

        if (pres_ns >= 40000 && pres_ns <= 300000)
        {
          presence = true;
          r += 2; // consume the presence pulse
        }
      }

      continue;
    }

    int bit;

    if (low_ns <= OW_ONE_MAX_NS)
    {
      bit = 1;
      short_lows++;
    }
    else if (low_ns >= OW_ZERO_MIN_NS && low_ns <= OW_ZERO_MAX_NS)
      bit = 0;
    else
    {
      out->errors++;
      continue;
    }

    if (nbits == 0)
      byte_start = scratch->pos[r];

    value |= bit << nbits; // LSB first

    if (++nbits == 8)
    {
      out->bytes[out->count] = (uint8_t)value;
      out->pos[out->count] = byte_start;
      out->end[out->count] = scratch->pos[r] + scratch->len[r];
      out->count++;

      value = 0;
      nbits = 0;
    }
  }

  // Bit slots without a reset are indistinguishable from random pulses, and
  // slow UART traffic can fake both the reset (a run of '0' bits) and the
  // zero slots. What UART can never produce is 1-Wire's other pulse
  // population: the 1-15 us '1' slots — require at least two of them.
  if (!seen_reset || short_lows < 2 || out->count == 0)
    return 0;

  out->rate = 60000; // nominal 60 us slot

  ow_analyze(out, at_reset, &g_ow);
  ow_build_info(&g_ow, presence, out->errors, out->info, sizeof(out->info));

  return out->count;
}
