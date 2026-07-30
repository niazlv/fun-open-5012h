/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for Flappy Bird.
 *
 * flappy_bird.c is #included rather than linked, so the checks can see the
 * scene behind the screen - the pipes, the bird, the state machine - while
 * still driving it the way the firmware does, through buttons and timer ticks.
 * See tests/gamestub.h for what stands in for the hardware.
 *
 * Build & run (no hardware needed), from the repository root:
 *   make test
 *
 * With a path argument it also writes that frame out as a .ppm:
 *   ./tests/build/flappy_test /tmp/flappy.ppm
 */

#include "gamestub.h"
#include "../src/apps/flappy_bird.c"

/*- Test plumbing -----------------------------------------------------------*/

static void advance(int ms)
{
    for (int i = 0; i < ms; i++)
    {
        tick_timers();
        flappy_bird_task();
    }
}

static void restart_app(void)
{
    flappy_bird_cleanup();
    flappy_bird_init();
}

/*- Artwork -----------------------------------------------------------------*/

static void t_sprites(void)
{
    static const sprite_t *const sprites[] =
        { &g_spr_bird, &g_spr_cloud, &g_spr_medal };
    static const char *const names[] = { "bird", "cloud", "medal" };

    printf("artwork:\n");
    check_sprites(sprites, names, ARRAY_SIZE(sprites));
}

/*- Difficulty --------------------------------------------------------------*/

static void t_ramp(void)
{
    int start_gap, start_speed;

    printf("difficulty:\n");

    g_difficulty = DIFF_NORMAL;
    g_ramp = true;
    g_score = 0;

    start_gap = pipe_gap();
    start_speed = pipe_speed();

    check("the gap starts where the difficulty says", start_gap,
        g_base_gap[DIFF_NORMAL]);

    g_score = RAMP_EVERY;
    check("and closes a step in", pipe_gap(), start_gap - RAMP_GAP);

    g_score = RAMP_EVERY * RAMP_SPEED_AT;
    check_true("the pipes come quicker with it", pipe_speed() > start_speed);

    g_score = 100000;
    check("the ramp stops", pipe_gap(),
        start_gap - RAMP_GAP * RAMP_STEPS);
    check_true("well clear of the bird", pipe_gap() > BIRD);

    g_ramp = false;
    check("switched off, nothing moves", pipe_gap(), start_gap);
    check("nor the speed", pipe_speed(), start_speed);

    g_ramp = true;
    g_score = 0;
}

/*- Rules -------------------------------------------------------------------*/

static void t_rules(void)
{
    printf("rules:\n");

    g_difficulty = DIFF_NORMAL;
    g_ramp = false;
    restart_app();

    // A pipe placed around the bird, with the bird in the middle of its gap
    g_state = ST_PLAY;
    g_pipes[0].x = BIRD_X;
    g_pipes[0].gap_y = 60;
    g_pipes[0].passed = false;

    for (int i = 1; i < MAX_PIPES; i++)
        g_pipes[i].x = LCD_WIDTH + 200;

    g_bird_y = 60 + pipe_gap() / 2 - BIRD / 2;
    check_true("through the middle of a gap is not a hit", !bird_hits_pipe());

    g_bird_y = 60 - BIRD;
    check_true("into the pipe above it is", bird_hits_pipe());

    g_bird_y = 60 + pipe_gap() + 2;
    check_true("and into the one below it is", bird_hits_pipe());

    // The corners of the sprite are transparent, so the box that kills is
    // inset: the sprite may overlap a collar by a pixel or two of nothing
    g_bird_y = 60 - BIRD_BODY + 1;
    check_true("a pixel of the sprite's own margin is not",
        !bird_hits_pipe());

    // Scoring, and that a pipe leaving the screen comes back a spacing behind
    // the last one rather than at the edge
    g_state = ST_PLAY;
    g_score = 0;
    g_bird_y = 60 + pipe_gap() / 2 - BIRD / 2;
    g_pipes[0].x = BIRD_X - PIPE_W - 1;
    g_pipes[0].passed = false;
    step_game();
    check("clearing a pipe scores", g_score, 1);

    {
        int rightmost = 0;

        g_pipes[0].x = -PIPE_W - PIPE_LIP_OVER - 4;

        for (int i = 1; i < MAX_PIPES; i++)
        {
            g_pipes[i].x = 200 + i * 40;

            if (g_pipes[i].x > rightmost)
                rightmost = g_pipes[i].x;
        }

        step_game();
        check_true("a respawned pipe keeps its distance",
            g_pipes[0].x >= rightmost + PIPE_SPACING - pipe_speed() - 1);
    }
}

/*- The ground and the medals -----------------------------------------------*/

static void t_ground_and_medals(void)
{
    printf("ground and medals:\n");

    check("nothing gets a medal", medal_for(0), 0);
    check("bronze", medal_for(MEDAL_BRONZE), 1);
    check("silver", medal_for(MEDAL_SILVER), 2);
    check("gold", medal_for(MEDAL_GOLD), 3);
    check("and gold is the top of it", medal_for(MEDAL_GOLD * 10), 3);

    g_difficulty = DIFF_NORMAL;
    restart_app();
    g_state = ST_PLAY;
    g_bird_y = SKY_H;           // already through the floor
    g_bird_v = 1.0f;
    step_game();
    check_true("the ground ends a run", ST_DYING == g_state);
    check_true("with the bird on it", g_bird_y <= SKY_H - BIRD);
}

/*- Persistence -------------------------------------------------------------*/

static void t_persistence(void)
{
    printf("best score:\n");

    config.flappy_high_score = 0;
    restart_app();

    g_state = ST_PLAY;
    g_score = 42;
    g_bird_y = SKY_H - BIRD;
    die();

    check("dying banks the score", g_best, 42);
    check_true("and says so", g_new_best);

    // The fall is what carries the state into the panel, and the panel is where
    // the score reaches the settings store
    advance(3000);

    check_true("the game over panel is up", ST_OVER == g_state);
    check("and the best score went to flash", config.flappy_high_score, 42);

    config.flappy_high_score = 0;
}

/*- A player ----------------------------------------------------------------*/

// Aims at the middle of the next gap and flaps when it is below it. Enough to
// keep a bird alive for a few hundred pipes, which is all this is for.
static bool t_should_flap(void)
{
    int gap = pipe_gap();
    int target = SKY_H / 2;
    int best = LCD_WIDTH * 2;

    for (int i = 0; i < MAX_PIPES; i++)
    {
        int ahead = g_pipes[i].x + PIPE_W - BIRD_X;

        if (ahead < 0 || ahead > best)
            continue;

        best = ahead;
        target = g_pipes[i].gap_y + gap / 2 - BIRD / 2;
    }

    return g_bird_y > target;
}

// The best score of the flight, not the score at the final frame: the bird
// dies eventually, the loop above restarts it, and reading g_score at a fixed
// frame number reads whichever run happens to be in progress there. That made
// the check a question about where the window falls - one pipe layout scored
// twelve, the next scored zero with the same autopilot, and neither said
// anything about the game.
static int t_fly(int frames)
{
    int best = 0;

    for (int f = 0; f < frames; f++)
    {
        if (ST_PLAY == g_state && t_should_flap())
            flappy_bird_buttons_handler(BTN_MODE);
        else if (ST_TITLE == g_state || ST_OVER == g_state)
            flappy_bird_buttons_handler(BTN_MODE);

        advance(FRAME_MS);

        if (g_score > best)
            best = g_score;
    }

    return best;
}

/*- Main --------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    printf("flappy bird\n\n");

    t_sprites();
    printf("\n");

    t_ramp();
    printf("\n");

    t_rules();
    printf("\n");

    t_ground_and_medals();
    printf("\n");

    t_persistence();
    printf("\n");

    printf("play:\n");
    g_difficulty = DIFF_NORMAL;
    g_ramp = true;
    restart_app();

    {
        int score = t_fly(1500);

        check_true("the player gets somewhere", score >= 5);
        check("nothing was drawn off the panel", g_offscreen_blits, 0);
        check_true("and it is still flying or trying to",
            ST_PLAY == g_state || ST_OVER == g_state || ST_DYING == g_state);
    }

    if (argc > 1)
    {
        while (ST_PLAY != g_state)
        {
            flappy_bird_buttons_handler(BTN_MODE);
            advance(FRAME_MS);
        }

        t_fly(120);
        write_ppm(argv[1]);
    }

    printf("\n%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED",
        g_failures);

    return g_failures ? 1 : 0;
}
