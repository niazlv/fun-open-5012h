/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for 2048.
 *
 * game2048.c is #included rather than linked, so the checks can set a board up
 * square by square and read the result back. See tests/gamestub.h for what
 * stands in for the hardware.
 *
 * Build & run (no hardware needed), from the repository root:
 *   make test
 *
 * With a path argument it also writes that frame out as a .ppm:
 *   ./tests/build/g2048_test /tmp/2048.ppm
 */

#include "gamestub.h"
#include "../src/apps/game2048.c"

/*- Test plumbing -----------------------------------------------------------*/

// A board written the way it looks: one exponent per square, row by row
static void t_set(int size, const uint8_t *cells)
{
    g_size = size;
    memcpy(g_board, cells, size * size);
    g_score = 0;
    g_over = false;
    g_won = false;
    g_can_undo = false;
    g_drawn_valid = false;
}

static bool t_equals(const uint8_t *want)
{
    return 0 == memcmp(g_board, want, g_size * g_size);
}

static void t_dump(const char *label, const uint8_t *want)
{
    printf("       %s\n       got ", label);

    for (int i = 0; i < g_size * g_size; i++)
        printf("%d%s", g_board[i], (i % g_size == g_size - 1) ? " / " : " ");

    printf("\n       want ");

    for (int i = 0; i < g_size * g_size; i++)
        printf("%d%s", want[i], (i % g_size == g_size - 1) ? " / " : " ");

    printf("\n");
}

// Slides without spawning anything, so the result is only the slide
static bool t_slide(int dir)
{
    bool moved = false;

    for (int l = 0; l < g_size; l++)
    {
        if (slide_line(dir, l))
            moved = true;
    }

    return moved;
}

static void t_check_board(const char *name, int dir, const uint8_t *cells,
    const uint8_t *want, int size)
{
    t_set(size, cells);
    t_slide(dir);

    if (t_equals(want))
    {
        printf("  PASS %s\n", name);
    }
    else
    {
        printf("  FAIL %s\n", name);
        t_dump(name, want);
        g_failures++;
    }
}

/*- The slide ---------------------------------------------------------------*/

static void t_rules(void)
{
    printf("the slide:\n");

    // Everything packs towards the wall it is pushed at
    {
        static const uint8_t in[]   = { 0,1,0,1,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
        static const uint8_t left[] = { 2,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

        t_check_board("two equal tiles merge", DIR_LEFT, in, left, 4);
    }

    // The rule that makes 2048 a game and not an accumulator
    {
        static const uint8_t in[]   = { 1,1,1,1,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
        static const uint8_t left[] = { 2,2,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

        t_check_board("a row of four makes two pairs, not one 8",
            DIR_LEFT, in, left, 4);
    }

    // A tile that has just been made cannot merge again in the same move
    {
        static const uint8_t in[]   = { 1,1,2,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
        static const uint8_t left[] = { 2,2,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

        t_check_board("a fresh tile does not merge again", DIR_LEFT, in, left, 4);
    }

    // Which end of the row a merge happens at depends on the direction
    {
        static const uint8_t in[]    = { 1,1,1,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
        static const uint8_t left[]  = { 2,1,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };
        static const uint8_t right[] = { 0,0,1,2,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

        t_check_board("three tiles left merge the leading pair",
            DIR_LEFT, in, left, 4);
        t_check_board("and right merge the trailing pair",
            DIR_RIGHT, in, right, 4);
    }

    // Columns, to prove the direction mapping and not just the row case
    {
        static const uint8_t in[]   = { 1,0,0,0,  1,0,0,0,  0,0,0,0,  3,0,0,0 };
        static const uint8_t up[]   = { 2,0,0,0,  3,0,0,0,  0,0,0,0,  0,0,0,0 };
        static const uint8_t down[] = { 0,0,0,0,  0,0,0,0,  2,0,0,0,  3,0,0,0 };

        t_check_board("a column packs upwards", DIR_UP, in, up, 4);
        t_check_board("and downwards", DIR_DOWN, in, down, 4);
    }

    // Every board size uses the same code, so the mapping has to hold at each
    {
        static const uint8_t in3[]  = { 1,1,0,  0,0,0,  0,0,0 };
        static const uint8_t out3[] = { 2,0,0,  0,0,0,  0,0,0 };
        static const uint8_t in6[]  = { 0,0,0,0,1,1,  0,0,0,0,0,0,  0,0,0,0,0,0,
                                        0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0 };
        static const uint8_t out6[] = { 0,0,0,0,0,2,  0,0,0,0,0,0,  0,0,0,0,0,0,
                                        0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0 };

        t_check_board("3x3 slides", DIR_LEFT, in3, out3, 3);
        t_check_board("6x6 slides", DIR_RIGHT, in6, out6, 6);
    }

    // Scoring is the value of what was made, not of what went into it
    {
        static const uint8_t in[] = { 1,1,2,2,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

        t_set(4, in);
        t_slide(DIR_LEFT);
        check("a 4 and an 8 score 12", g_score, 12);
    }
}

/*- Moves, undo and the end -------------------------------------------------*/

static void t_moves(void)
{
    printf("moves:\n");

    // A press that shifts nothing is not a move: no tile appears for it, which
    // is the difference between a game and a slot machine
    {
        static const uint8_t packed[] = { 1,2,1,2,  2,1,2,1,  1,2,1,2,  2,1,2,1 };
        int before;

        t_set(4, packed);
        before = free_cells();
        do_move(DIR_LEFT);

        check("a move that changes nothing spawns nothing",
            free_cells(), before);
        check_true("and leaves the score alone", 0 == g_score);
        check_true("and offers no undo", !g_can_undo);
    }

    // A real move spawns exactly one
    {
        static const uint8_t in[] = { 1,1,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

        t_set(4, in);
        do_move(DIR_LEFT);

        check("a real move spawns one tile", 16 - free_cells(), 2);
        check_true("and can be taken back", g_can_undo);

        undo_move();
        check("undo puts the board back", free_cells(), 14);
        check("and the score with it", g_score, 0);
        check_true("once, and not twice", !g_can_undo);
    }

    // The end: full board, no two neighbours alike
    {
        static const uint8_t dead[] = { 1,2,1,2,  2,1,2,1,  1,2,1,2,  2,1,2,1 };
        static const uint8_t alive[] = { 1,2,1,2,  2,1,2,1,  1,2,1,2,  2,1,2,2 };

        t_set(4, dead);
        check_true("a locked board has no moves", !moves_left());

        t_set(4, alive);
        check_true("one matching pair is enough to go on", moves_left());
    }

    // 2048 is a milestone, not the end
    {
        static const uint8_t nearly[] = { 10,10,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 };

        t_set(4, nearly);
        do_move(DIR_LEFT);

        check_true("2048 is reached", board_has(WIN_EXP));
        check_true("and the game carries on", !g_over);
    }
}

/*- Geometry ----------------------------------------------------------------*/

// Every board size has to land inside its panel, whatever the division leaves
// over. A tile hanging off the edge is not a rounding error on this display -
// lcd_draw_buf does not clip, and lcd_fill_rect silently eats the overflow.
static void t_geometry(void)
{
    printf("geometry:\n");

    for (int n = MIN_SIZE; n <= MAX_SIZE; n++)
    {
        char label[64];
        int last;

        g_size = n;
        last = cell_origin(n - 1) + cell_size();

        snprintf(label, sizeof(label), "%dx%d: the grid fits its panel", n, n);
        check_true(label, cell_origin(0) > 0 && last <= BOARD_PX);

        snprintf(label, sizeof(label), "%dx%d: tiles do not touch", n, n);
        check_true(label, cell_origin(1) - (cell_origin(0) + cell_size()) > 0);

        snprintf(label, sizeof(label), "%dx%d: a tile is worth drawing on", n, n);
        check_true(label, cell_size() >= 24);
    }
}

/*- Play --------------------------------------------------------------------*/

// Left, up, left, up... which is the corner strategy every 2048 player finds,
// and enough to drive a few hundred real moves through the whole machine.
// The clock runs between them, so every frame of the slide and the settle is
// drawn for real - that is what puts the animation under the off-panel check.
static void t_play(int size, int moves)
{
    static const int cycle[] = { DIR_LEFT, DIR_UP, DIR_LEFT, DIR_DOWN };

    g_size = size;
    new_game();
    draw_all();

    for (int i = 0; i < moves && !g_over; i++)
    {
        do_move(cycle[i % ARRAY_SIZE(cycle)]);

        for (int ms = 0; ms < (SLIDE_STEPS + POP_STEPS + 1) * TICK_MS; ms++)
        {
            tick_timers();
            game2048_task();
        }
    }
}

/*- Main --------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    printf("2048\n\n");

    game2048_init();

    t_rules();
    printf("\n");

    t_moves();
    printf("\n");

    t_geometry();
    printf("\n");

    printf("play:\n");
    {
        int reached = 0;

        for (int n = MIN_SIZE; n <= MAX_SIZE; n++)
        {
            t_play(n, 400);

            if (g_score > reached)
                reached = g_score;
        }

        check_true("a few hundred moves get somewhere", reached > 200);
        check_true("and the animation always settles", 0 == g_anim_phase);
        check("nothing was drawn off the panel", g_offscreen_blits, 0);
    }

    // A frame to look at, on whichever board a second argument names
    if (argc > 1)
    {
        int size = (argc > 2) ? atoi(argv[2]) : 4;

        if (size < MIN_SIZE || size > MAX_SIZE)
            size = 4;

        t_play(size, 60 * size);
        draw_all();
        write_ppm(argv[1]);
    }

    printf("\n%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED",
        g_failures);

    return g_failures ? 1 : 0;
}
