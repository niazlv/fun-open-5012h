/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Runs a BK tape image on the host, with the same emulator the device does.
 *
 * The device is a poor place to find out that a program does not work: the
 * screen says "THE MACHINE STOPPED" and one address, and getting a second
 * opinion means reflashing. This runs the identical src/bk sources against the
 * identical 64 KB, writes the screen out as a .ppm, and can print every
 * instruction on the way - so the question "why did it stop" is answerable
 * with a diff instead of a guess.
 *
 * Not part of `make test`: it needs a tape image, and those are somebody's
 * software.
 *
 *   make -C tests bkhost
 *   ./tests/build/bk_host game.bin --frames 200 --ppm /tmp/bk.ppm
 *   ./tests/build/bk_host game.bin --trace 40
 *   ./tests/build/bk_host game.bin --rom bk10mon.rom --frames 500
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "lcd.h"
#include "bk_config.h"
#include "bk_cpu.h"
#include "bk_mem.h"
#include "bk_io.h"
#include "bk_video.h"
#include "bk_load.h"
#include "bk_disasm.h"

/*- The panel ---------------------------------------------------------------*/
#define PANEL_W     320
#define PANEL_H     240

static uint16_t g_panel[PANEL_H][PANEL_W];

void lcd_set_font(const Font *f) { (void)f; }
void lcd_set_color(int bg, int fg) { (void)bg; (void)fg; }
void lcd_puts(int x, int y, const char *s) { (void)x; (void)y; (void)s; }
void lcd_draw_rect(int x, int y, int w, int h, int c)
{
    (void)x; (void)y; (void)w; (void)h; (void)c;
}

void lcd_fill_rect(int x, int y, int w, int h, int color)
{
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            if (x + i >= 0 && x + i < PANEL_W && y + j >= 0 && y + j < PANEL_H)
                g_panel[y + j][x + i] = (uint16_t)color;
}

void lcd_draw_indexed(int x, int y, int w, int h, const uint8_t *pix,
    const uint16_t *palette)
{
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            if (x + i >= 0 && x + i < PANEL_W && y + j >= 0 && y + j < PANEL_H)
                g_panel[y + j][x + i] = palette[pix[j * w + i]];
}

/*- Plumbing ----------------------------------------------------------------*/
static uint8_t g_ram[BK_MEM_SIZE];
static uint8_t g_blit[BK_VIDEO_BUFFER_SIZE];

static uint8_t *slurp(const char *path, long *size)
{
    FILE *f = fopen(path, "rb");
    uint8_t *buf;
    long n;

    if (NULL == f)
    {
        fprintf(stderr, "cannot open %s\n", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);

    buf = malloc((size_t)n);

    if (NULL == buf || 1 != fread(buf, (size_t)n, 1, f))
    {
        fprintf(stderr, "cannot read %s\n", path);
        fclose(f);
        free(buf);
        return NULL;
    }

    fclose(f);
    *size = n;

    return buf;
}

static void write_ppm(const char *path)
{
    FILE *f = fopen(path, "wb");

    if (NULL == f)
    {
        fprintf(stderr, "cannot write %s\n", path);
        return;
    }

    fprintf(f, "P6\n%d %d\n255\n", PANEL_W, PANEL_H);

    for (int y = 0; y < PANEL_H; y++)
    {
        for (int x = 0; x < PANEL_W; x++)
        {
            uint16_t c = g_panel[y][x];
            uint8_t rgb[3];

            // 5-6-5 back out to eight bits a channel
            rgb[0] = (uint8_t)(((c >> 11) & 0x1f) * 255 / 31);
            rgb[1] = (uint8_t)(((c >> 5) & 0x3f) * 255 / 63);
            rgb[2] = (uint8_t)((c & 0x1f) * 255 / 31);

            fwrite(rgb, 3, 1, f);
        }
    }

    fclose(f);
    printf("wrote %s\n", path);
}

//-----------------------------------------------------------------------------
// What the screen has on it, without looking at it: how many of the 16 KB are
// not zero, and how many distinct colours the picture uses. A program that has
// drawn nothing and a program that has drawn something are then distinguishable
// from a terminal.
static void describe_screen(void)
{
    unsigned nonzero = 0;
    unsigned counts[4] = { 0, 0, 0, 0 };

    for (unsigned i = 0; i < BK_VRAM_SIZE; i++)
    {
        uint8_t b = g_ram[BK_VRAM_BASE + i];

        if (b)
            nonzero++;

        for (int p = 0; p < 4; p++)
            counts[(b >> (2 * p)) & 3]++;
    }

    printf("screen:   %u of %u bytes written\n", nonzero, BK_VRAM_SIZE);
    printf("colours:  %u black, %u one, %u two, %u three\n",
        counts[0], counts[1], counts[2], counts[3]);
    printf("scroll:   %06o\n", bk_io_scroll());
    printf("palette:  %u\n", bk_io_palette());
}

//-----------------------------------------------------------------------------
static void disassemble(uint16_t pc, int count)
{
    char text[48];

    for (int i = 0; i < count; i++)
    {
        int len = bk_disasm(pc, text, sizeof(text));

        printf("  %06o: %s\n", pc, text);
        pc = (uint16_t)(pc + len);
    }
}

/*- Main --------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    const char *tape = NULL, *rom = NULL, *ppm = NULL;
    int frames = 100;
    int trace = 0;
    int top = 8;
    uint8_t key = 0;
    bool squash = false;
    bool mono = false, window = false;
    long size;
    uint8_t *data;
    uint32_t addr, length;

    for (int i = 1; i < argc; i++)
    {
        if (0 == strcmp(argv[i], "--rom") && i + 1 < argc)
            rom = argv[++i];
        else if (0 == strcmp(argv[i], "--ppm") && i + 1 < argc)
            ppm = argv[++i];
        else if (0 == strcmp(argv[i], "--frames") && i + 1 < argc)
            frames = atoi(argv[++i]);
        else if (0 == strcmp(argv[i], "--trace") && i + 1 < argc)
            trace = atoi(argv[++i]);
        else if (0 == strcmp(argv[i], "--mono"))
            mono = true;
        else if (0 == strcmp(argv[i], "--window"))
            window = true;
        else if (0 == strcmp(argv[i], "--squash"))
            squash = true;
        else if (0 == strcmp(argv[i], "--top") && i + 1 < argc)
            top = atoi(argv[++i]);
        else if (0 == strcmp(argv[i], "--key") && i + 1 < argc)
            key = (uint8_t)strtoul(argv[++i], NULL, 0);
        else if ('-' != argv[i][0])
            tape = argv[i];
    }

    if (NULL == tape)
    {
        fprintf(stderr,
            "usage: bk_host <tape.bin> [--rom mon.rom] [--ppm out.ppm]\n"
            "                          [--frames n] [--trace n]\n"
            "                          [--squash] [--top n] [--key code]\n");
        return 2;
    }

    bk_mem_attach(g_ram);
    bk_video_set_buffer(g_blit);
    bk_io_reset();

    /*
     * The tape header, read here rather than through bk_load_bin() - that one
     * reads through spifs, which is the device's flash and not a file.
     */
    data = slurp(tape, &size);

    if (NULL == data)
        return 1;

    if (size < 4)
    {
        fprintf(stderr, "%s is too short to be a tape image\n", tape);
        return 1;
    }

    addr = (uint32_t)data[0] | ((uint32_t)data[1] << 8);
    length = (uint32_t)data[2] | ((uint32_t)data[3] << 8);

    printf("tape:     %s\n", tape);
    printf("load:     %06o, %u bytes (file has %ld)\n", addr, length, size - 4);

    if (length + 4 > (uint32_t)size)
    {
        fprintf(stderr, "header claims more bytes than the file has\n");
        return 1;
    }

    if (addr + length > BK_ROM_BASE)
    {
        fprintf(stderr, "the program would run past the top of RAM\n");
        return 1;
    }

    memcpy(g_ram + addr, data + 4, length);
    printf("occupies: %06o..%06o\n", addr, addr + length - 1);

    if (rom)
    {
        long rsize;
        uint8_t *rdata = slurp(rom, &rsize);

        if (NULL == rdata)
            return 1;

        bk_mem_load_rom(BK_MON_BASE, rdata, (uint32_t)rsize);
        printf("rom:      %s, %ld bytes at %06o\n", rom, rsize, BK_MON_BASE);
    }
    else
    {
        bk_load_stub_rom((uint16_t)addr);
        printf("rom:      the stand-in\n");
    }

    printf("\nfirst instructions:\n");
    disassemble((uint16_t)addr, 8);

    bk_load_boot();
    bk_video_init();
    bk_video_set_fit(squash ? BK_FIT_SQUASH : BK_FIT_CROP);
    bk_video_set_screen(mono ? BK_SCREEN_MONO : BK_SCREEN_COLOR);
    bk_video_set_mono(window ? BK_MONO_WINDOW : BK_MONO_WHOLE);
    bk_video_set_top(top);

    if (trace > 0)
    {
        char text[48];

        printf("\ntrace:\n");

        for (int i = 0; i < trace && NULL == bk_cpu.stopped; i++)
        {
            bk_disasm(bk_cpu.r[7], text, sizeof(text));
            printf("  %06o: %-28s  R0=%06o R1=%06o R2=%06o SP=%06o %c%c%c%c\n",
                bk_cpu.r[7], text, bk_cpu.r[0], bk_cpu.r[1], bk_cpu.r[2],
                bk_cpu.r[6],
                (bk_cpu.psw & PSW_N) ? 'N' : '-',
                (bk_cpu.psw & PSW_Z) ? 'Z' : '-',
                (bk_cpu.psw & PSW_V) ? 'V' : '-',
                (bk_cpu.psw & PSW_C) ? 'C' : '-');
            bk_cpu_step();
        }
    }

    // One frame is one 50 Hz tick, which is the clock a BK program keeps time by
    for (int i = 0; i < frames && NULL == bk_cpu.stopped; i++)
    {
        bk_io_frame();
        bk_cpu_run(BK_CPU_HZ / 50);
        bk_io_advance(20000);

        // A key, offered from a quarter of the way in and kept down after
        // that, so a program waiting on one gets past its title screen
        if (key && i >= frames / 4)
            bk_io_key(key, false);
    }

    bk_video_draw(true);

    printf("\nafter %d frames:\n", frames);
    printf("stopped:  %s\n", bk_cpu.stopped ? bk_cpu.stopped : "no, still running");
    printf("PC:       %06o    SP %06o    PSW %06o\n",
        bk_cpu.r[7], bk_cpu.r[6], bk_cpu.psw);
    printf("last trap through %06o\n", bk_cpu.last_trap);
    printf("insns:    %u in %u cycles\n", bk_cpu.insns, bk_cpu.cycles);
    printf("unknown I/O reads: %u\n", bk_io_unknown);
    printf("\nat the program counter:\n");
    disassemble(bk_cpu.stopped ? bk_cpu.pc_insn : bk_cpu.r[7], 6);
    printf("\n");
    describe_screen();

    if (ppm)
        write_ppm(ppm);

    return 0;
}
