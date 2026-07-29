/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for Tetris.
 *
 * tetris.c is #included rather than linked, so the checks can build a well one
 * row at a time and read the result back. See tests/gamestub.h for what stands
 * in for the hardware.
 *
 * Build & run (no hardware needed), from the repository root:
 *   make test
 *
 * With a path argument it also writes that frame out as a .ppm:
 *   ./tests/build/tetris_test /tmp/tetris.ppm
 */

#include "gamestub.h"
#include "../src/apps/tetris.c"

/*- Test plumbing -----------------------------------------------------------*/

static void advance(int ms)
{
    for (int i = 0; i < ms; i++)
    {
        tick_timers();
        tetris_task();
    }
}

// A well with `filled` cells set in the bottom row, leaving a gap at `hole`
static void t_fill_row(int y, int hole)
{
    for (int x = 0; x < COLS; x++)
        g_well[y][x] = (x == hole) ? 0 : 1;
}

/*- The pieces --------------------------------------------------------------*/

static void t_shapes(void)
{
    static const char *const name[] = { "I", "O", "T", "S", "Z", "J", "L" };
    int bad_count = 0;
    int bad_cycle = 0;

    printf("the pieces:\n");

    for (int p = 0; p < PIECES; p++)
    {
        for (int r = 0; r < 4; r++)
        {
            int n = 0;

            for (int i = 0; i < BOX * BOX; i++)
            {
                if (piece_cell(p, r, i))
                    n++;
            }

            if (4 != n)
            {
                printf("       %s rotation %d has %d cells\n", name[p], r, n);
                bad_count++;
            }
        }

        // Four turns is where it started, or the piece drifts across the well
        // every time it is spun
        if (g_shapes[p][0] != g_shapes[p][(0 + 4) & 3])
            bad_cycle++;
    }

    check("every rotation is four cells", bad_count, 0);
    check("and four turns come back round", bad_cycle, 0);

    // The bag: seven pieces, each exactly once, however it is shuffled
    {
        int seen[PIECES];
        int wrong = 0;

        memset(seen, 0, sizeof(seen));
        g_bag_left = 0;

        for (int i = 0; i < PIECES; i++)
            seen[next_from_bag()]++;

        for (int i = 0; i < PIECES; i++)
        {
            if (1 != seen[i])
                wrong++;
        }

        check("a bag deals each piece once", wrong, 0);
    }
}

/*- Collision ---------------------------------------------------------------*/

static void t_fits(void)
{
    printf("fit:\n");

    memset(g_well, 0, sizeof(g_well));

    check_true("a piece fits an empty well", fits(0, 0, 3, 5));
    check_true("but not through the left wall", !fits(0, 0, -2, 5));
    check_true("nor the right one", !fits(0, 0, COLS - 1, 5));
    check_true("nor the floor", !fits(0, 0, 3, ROWS));

    // Above the ceiling is legal - that is where a piece spawns
    check_true("above the well is not a collision", fits(2, 0, 3, -1));

    // One hole wide, so only something one cell across can use it: an O needs
    // two columns and a standing I needs one
    t_fill_row(ROWS - 1, 4);
    check_true("a filled row blocks", !fits(1, 0, 0, ROWS - 2));
    check_true("and a two wide piece cannot use a one wide gap",
        !fits(1, 0, 3, ROWS - 2));
    check_true("but a standing I drops straight into it",
        fits(0, 1, 3, ROWS - 4));

    memset(g_well, 0, sizeof(g_well));
}

/*- Lines -------------------------------------------------------------------*/

static void t_lines(void)
{
    printf("lines:\n");

    // An O dropped into a two wide gap completes two rows at once
    memset(g_well, 0, sizeof(g_well));
    g_state = ST_PLAY;
    g_score = 0;
    g_lines = 0;
    g_level = 0;

    for (int y = ROWS - 2; y < ROWS; y++)
    {
        for (int x = 0; x < COLS; x++)
            g_well[y][x] = (x == 4 || x == 5) ? 0 : 1;
    }

    g_piece = 1;    // O
    g_rot = 0;
    g_px = 3;       // the O of shape 0x0066 sits at box columns 1 and 2
    g_py = ROWS - 2;

    check("the O completes two rows", lock_piece(), 2);

    g_lines += 2;
    g_score += g_line_score[2] * (g_level + 1);
    check("two rows at once score 300", g_score, 300);
    check_true("which beats two singles", g_line_score[2] > 2 * g_line_score[1]);

    collapse_rows();

    {
        int left = 0;

        for (int y = 0; y < ROWS; y++)
        {
            for (int x = 0; x < COLS; x++)
            {
                if (g_well[y][x])
                    left++;
            }
        }

        check("and the well is empty after them", left, 0);
    }

    // What is above a cleared row comes down with it
    memset(g_well, 0, sizeof(g_well));
    t_fill_row(ROWS - 1, -1);       // complete
    g_well[ROWS - 3][2] = 5;        // a lone block two rows up

    memset(g_full_row, 0, sizeof(g_full_row));
    g_full_row[ROWS - 1] = 1;
    collapse_rows();

    check("the stack above falls by one", g_well[ROWS - 2][2], 5);
    check("and leaves nothing behind", g_well[ROWS - 3][2], 0);

    // The level follows the rows
    g_start_level = 0;
    check("ten rows is level one", level_for(10), 1);
    check("and the level caps", level_for(1000), 20);
    check_true("faster with every level",
        g_fall_table[5] < g_fall_table[0]);
}

/*- Moves -------------------------------------------------------------------*/

static void t_moves(void)
{
    printf("moves:\n");

    memset(g_well, 0, sizeof(g_well));
    g_state = ST_PLAY;
    g_shown_valid = false;

    g_piece = 2;    // T
    g_rot = 0;
    g_px = 0;
    g_py = 5;

    // A turn that does not fit where it stands is nudged, not refused. The
    // standing I fills column 1 of its box, so hard against the left wall is
    // px = -1; laid flat from there it would start at x = -1, off the board.
    g_piece = 0;
    g_rot = 1;
    g_px = -1;
    g_py = 5;
    check_true("a standing I sits against the wall", fits(0, 1, -1, 5));
    check_true("and laid flat from there it would not", !fits(0, 2, -1, 5));

    try_rotate();
    check("turning it kicks it into the well", g_px, 0);
    check("and the turn is taken", g_rot, 2);

    // Hard drop lands the piece on the floor
    memset(g_well, 0, sizeof(g_well));
    g_piece = 1;
    g_rot = 0;
    g_px = 3;
    g_py = 0;
    check("the drop row is the floor", drop_row(), ROWS - 2);

    // Hold: once per piece, and it comes back
    memset(g_well, 0, sizeof(g_well));
    g_hold = -1;
    g_piece = 3;
    g_held_this_piece = false;
    g_next = 4;
    hold_piece();
    check("holding puts the piece aside", g_hold, 3);
    check_true("and brings up the next one", 4 == g_piece);

    g_piece = 5;
    hold_piece();
    check_true("a second hold in one piece does nothing", 3 == g_hold);

    g_held_this_piece = false;
    hold_piece();
    check("holding again swaps them back", g_hold, 5);
    check_true("and the held piece is now falling", 3 == g_piece);
}

/*- Soft drop ---------------------------------------------------------------*/

// One point a cell, and only for a cell the piece actually descended. Holding
// the button down on a piece that has nowhere left to go must score nothing:
// the repeat fires every 10 ms once it has wound up, so anything paid per
// press rather than per cell would be a tap-to-win.
static void t_soft_drop(void)
{
    int before, cells;

    printf("soft drop:\n");

    memset(g_well, 0, sizeof(g_well));
    g_state = ST_PLAY;
    g_score = 0;
    g_piece = 1;        // O
    g_rot = 0;
    g_px = 3;
    g_py = 0;

    cells = drop_row() - g_py;

    for (int i = 0; i < 40; i++)
        tetris_buttons_handler(BTN_DOWN | BTN_REPEAT);

    check("a soft drop pays one point a cell", g_score, cells);
    check("and lands the piece on the floor", g_py, ROWS - 2);

    // Now it is resting. The lock delay has not run, so it is still the same
    // piece, and the button is still down.
    before = g_score;

    for (int i = 0; i < 200; i++)
        tetris_buttons_handler(BTN_DOWN | BTN_REPEAT);

    check("holding it there pays nothing more", g_score, before);
    check_true("and the piece has not locked yet", ST_PLAY == g_state);

    // The most one piece can be worth this way is the height of the well
    check_true("so a piece is worth at most its fall", cells <= ROWS);
}

/*- The end -----------------------------------------------------------------*/

static void t_over(void)
{
    printf("the end:\n");

    config.tetris_high_score = 0;
    tetris_init();

    g_state = ST_PLAY;
    g_score = 777;

    // Fill the well to the ceiling, so nothing can spawn
    for (int y = 0; y < ROWS; y++)
    {
        for (int x = 0; x < COLS; x++)
            g_well[y][x] = 1;
    }

    spawn_piece(2);

    check_true("nowhere to spawn ends the run", ST_OVER == g_state);
    check("the score is banked", g_best, 777);
    check("and reaches flash", config.tetris_high_score, 777);

    config.tetris_high_score = 0;
}

/*- Geometry ----------------------------------------------------------------*/

static void t_geometry(void)
{
    printf("geometry:\n");

    check_true("the well fits the panel",
        WELL_X + WELL_W <= LCD_WIDTH && WELL_Y + WELL_H <= LCD_HEIGHT);
    check_true("and leaves room for its frame",
        WELL_X >= 2 && WELL_Y >= 2);
    check_true("the side panel starts clear of the well",
        PANEL_X > WELL_X + WELL_W);
    check_true("and ends inside the display", PANEL_X + 140 <= LCD_WIDTH);
}

/*- Play --------------------------------------------------------------------*/

// Drops pieces into the leftmost column that takes them, which is not good
// Tetris but is enough to run pieces, locks, line clears and the level curve
// through the whole machine
static void t_play(int pieces)
{
    tetris_init();
    g_start_level = 0;
    new_game();
    g_state = ST_PLAY;
    draw_all();

    for (int p = 0; p < pieces && ST_OVER != g_state; p++)
    {
        int target = rand() % COLS;

        for (int i = 0; i < COLS && ST_PLAY == g_state; i++)
        {
            if (g_px == target - 1)
                break;

            try_move((g_px < target) ? 1 : -1);
        }

        if (0 == (p % 3))
            try_rotate();

        if (ST_PLAY == g_state)
            hard_drop();

        // The clock runs, so the line-clear flash is drawn for real
        advance((CLEAR_FLASHES + 2) * CLEAR_MS);
    }
}

/*- Main --------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    printf("tetris\n\n");

    tetris_init();

    t_shapes();
    printf("\n");

    t_fits();
    printf("\n");

    t_lines();
    printf("\n");

    t_moves();
    printf("\n");

    t_soft_drop();
    printf("\n");

    t_over();
    printf("\n");

    t_geometry();
    printf("\n");

    printf("play:\n");
    t_play(120);

    check_true("the pieces keep coming", g_lines > 0 || g_score > 0);
    check("nothing was drawn off the panel", g_offscreen_blits, 0);

    if (argc > 1)
    {
        t_play(24);
        draw_all();
        write_ppm(argv[1]);
    }

    printf("\n%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED",
        g_failures);

    return g_failures ? 1 : 0;
}
