# SPI NOR access over SWD, without any help from the firmware.
#
# The serial flash on this board hangs off the MCU and nothing else, so the PC
# cannot reach it directly - there is no USB, and the connector that looks like
# one carries SWDIO/SWCLK. What there is instead: the debugger can halt the
# core and use its SPI0 peripheral as a shift register. That is what this does.
#
# It costs three SWD transactions per byte, so it runs at roughly a kilobyte a
# second: right for a JEDEC id, a directory, a 16 KB monitor ROM or a rescue
# read of something you are about to overwrite, and wrong for a megabyte. For
# payloads that size the firmware has to do the shifting - see the mailbox
# loader.
#
# The core is halted for the duration and resumed at the end, so the scope
# stops sweeping while this runs.
#
# usage, all offsets and lengths in bytes:
#   openocd -f interface/cmsis-dap.cfg -c "transport select swd" \
#           -f target/stm32f4x.cfg -f tools/spiflash.tcl \
#           -c "spi_init; spi_id; spi_dump 0 256; shutdown"
#
#   ... -c "spi_init; spi_write_file 0x1000 bk/monitor.rom; shutdown"
#   ... -c "spi_init; spi_read_file 0x7F0000 65536 factory.bin; shutdown"

set SPI0_STAT 0x40013008
set SPI0_DATA 0x4001300C
set GPIOA_BOP 0x40020018
set GPIOA_BC  0x40020028

# PA3, active low. flash_init has already put the bus in the right mode, so
# this only has to frame commands.
proc cs_low  {} { global GPIOA_BC;  mww $GPIOA_BC  0x00000008 }
proc cs_high {} { global GPIOA_BOP; mww $GPIOA_BOP 0x00000008 }

proc xfer {byte} {
  global SPI0_STAT SPI0_DATA
  mww $SPI0_DATA $byte
  for {set i 0} {$i < 100} {incr i} {
    if {[expr {[mrw $SPI0_STAT] & 1}]} break
  }
  return [expr {[mrw $SPI0_DATA] & 0xff}]
}

proc cmd_addr {cmd addr} {
  cs_low
  xfer $cmd
  xfer [expr {($addr >> 16) & 0xff}]
  xfer [expr {($addr >> 8) & 0xff}]
  xfer [expr {$addr & 0xff}]
}

proc spi_init {} {
  init
  halt
}

# The latch clears itself after every program and erase, so this is per
# operation and not once at the start.
proc write_enable {} {
  cs_low
  xfer 0x06
  cs_high
}

proc wait_ready {{limit 2000}} {
  for {set i 0} {$i < $limit} {incr i} {
    cs_low
    xfer 0x05
    set status [xfer 0]
    cs_high
    if {($status & 1) == 0} { return 1 }
  }
  echo "spiflash: still busy after $limit polls"
  return 0
}

proc spi_id {} {
  cs_low
  xfer 0x9f
  set a [xfer 0]
  set b [xfer 0]
  set c [xfer 0]
  cs_high
  echo [format "JEDEC %02X %02X %02X   %d MB" $a $b $c [expr {(1 << $c) >> 20}]]
  return [list $a $b $c]
}

proc spi_read {addr count} {
  cmd_addr 0x03 $addr
  set out {}
  for {set i 0} {$i < $count} {incr i} { lappend out [xfer 0] }
  cs_high
  return $out
}

proc spi_dump {addr count} {
  set bytes [spi_read $addr $count]
  for {set i 0} {$i < $count} {incr i 16} {
    set line ""
    set text ""
    for {set j 0} {$j < 16 && $i + $j < $count} {incr j} {
      set b [lindex $bytes [expr {$i + $j}]]
      append line [format "%02X " $b]
      append text [expr {($b >= 32 && $b < 127) ? [format %c $b] : "."}]
    }
    echo [format "%08X: %-48s %s" [expr {$addr + $i}] $line $text]
  }
}

# Bytes as hex on one line, for a caller that is going to parse them. OpenOCD
# embeds Jim Tcl, which has no binary channel support at all - no `binary
# scan`, no reliable byte-for-byte file read - so everything that crosses the
# boundary to the host crosses it as text. tools/spiflash.py does the
# converting on the other side.
proc spi_read_hex {addr count} {
  set out ""
  foreach b [spi_read $addr $count] { append out [format "%02X" $b] }
  echo "HEX $addr $out"
}

proc spi_erase_sector {addr} {
  write_enable
  cmd_addr 0x20 $addr
  cs_high
  wait_ready
}

proc spi_erase_chip {} {
  write_enable
  cs_low
  xfer 0xc7
  cs_high
  echo "chip erase issued - tens of seconds"
  wait_ready 200000
}

# Programming can only clear bits and stops at the end of each 256 byte page,
# so this splits on page boundaries and expects the target to be erased.
proc spi_write {addr bytes} {
  set n [llength $bytes]
  set done 0
  while {$done < $n} {
    set here [expr {$addr + $done}]
    set room [expr {256 - ($here % 256)}]
    set chunk [expr {($n - $done) < $room ? ($n - $done) : $room}]
    write_enable
    cmd_addr 0x02 $here
    for {set i 0} {$i < $chunk} {incr i} {
      xfer [lindex $bytes [expr {$done + $i}]]
    }
    cs_high
    if {![wait_ready]} { return 0 }
    incr done $chunk
  }
  return 1
}

# ---------------------------------------------------------------------------
# The fast path: the firmware does the shifting.
#
# Everything above halts the core and drives SPI0 by hand, which costs three
# SWD transactions per byte. The debugger is however very good at putting a
# block of bytes into SRAM - that is how it programs the internal flash - so
# the SPI Flash Loader application on the device exposes a mailbox there and
# does the SPI side itself, at about fifty times the rate.
#
# The core keeps running throughout: it has to, since it is the thing
# executing the commands.

set MB_BASE 0x20000000
set MB_MAGIC 0x4C495053

proc mb_addr {off} { global MB_BASE; return [expr {$MB_BASE + $off}] }

proc mb_init {} {
  global MB_MAGIC
  init
  set magic [mrw [mb_addr 0]]
  if {$magic != $MB_MAGIC} {
    echo [format "no mailbox at %08X (read %08X)" [mb_addr 0] $magic]
    echo "open SPI Flash Loader on the device first"
    shutdown error
  }
  echo "mailbox ready"
}

# Fields are written before the command word and the command word last, so the
# firmware can never act on a half-built request.
proc mb_run {cmd addr len} {
  mww [mb_addr 8]  $addr
  mww [mb_addr 12] $len
  mww [mb_addr 16] 1        ;# status = busy, so a stale OK is not mistaken
  mww [mb_addr 4]  $cmd

  for {set i 0} {$i < 100000} {incr i} {
    set status [mrw [mb_addr 16]]
    if {$status > 1} {
      if {$status == 2} { return [mrw [mb_addr 20]] }
      echo [format "command %d failed at %08X (status %d)" $cmd $addr $status]
      shutdown error
    }
  }

  echo "device did not answer"
  shutdown error
}

# Reads through the device and hands the block back out of SRAM, which is the
# same trick as the write path in reverse: the slow part of a debugger read is
# the per-byte round trip, and this has none - one command, then one block
# transfer out of the mailbox buffer.
proc mb_read {addr len path} {
  mb_run 3 $addr $len
  dump_image $path [mb_addr 32] $len
}

# Rebuilds the file table on the device and hands it back. The scan is 1952
# sector heads; doing it from this side would be that many round trips.
proc mb_scan {path} {
  set n [mb_run 6 0 0]
  if {$n > 0} { dump_image $path [mb_addr 32] [expr {$n * 48}] }
  echo [format "FILES %d FREE %d" $n [mrw [mb_addr 8]]]
}

proc mb_crc {addr len} {
  echo [format "CRC %08X" [mb_run 4 $addr $len]]
}

# The override for the reserved top of the part, where the settings store
# lives. Deliberately awkward.
proc mb_unsafe {} { mww [mb_addr 28] 0x0FF77E57 }

# ---------------------------------------------------------------------------
# The other half of the text boundary: one hex string in, one page program out.
proc spi_write_hex {addr hex} {
  set bytes {}
  for {set i 0} {$i < [string length $hex]} {incr i 2} {
    lappend bytes [expr {"0x[string range $hex $i [expr {$i + 1}]]"}]
  }

  if {[spi_write $addr $bytes]} {
    echo [format "wrote %d bytes at %08X" [llength $bytes] $addr]
  }
}
