/*
 * Copyright (C) 1993-1996 Id Software, Inc.
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Derived from the DOOM source release (linuxdoom-1.10), published by
 * id Software under the GNU General Public License. This program is free
 * software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any
 * later version. It is distributed WITHOUT ANY WARRANTY; see doom/COPYING.
 *
 * The player: acceleration, friction, sliding and the view height.
 *
 * These are the original's numbers - thrust of forwardmove*2048 per tic, a
 * friction of 0.90625 applied every tic, a 24-unit step limit - and the tic
 * rate is the original's 35 Hz, so walking speed does not depend on how fast
 * the device happens to render.
 *
 * It takes a ticcmd rather than reading buttons, so the device shell owns the
 * key mapping and the host harness can drive exactly the same simulation from
 * a script. What the demo shows is then what the hardware does.
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom.h"

/*- Definitions -------------------------------------------------------------*/
#define FORWARD_WALK        0x19
#define FORWARD_RUN         0x32
#define SIDE_WALK           0x18
#define SIDE_RUN            0x28
#define TURN_WALK           640
#define TURN_RUN            1280
#define FRICTION            0xe800
#define STOPSPEED           0x1000
#define MAXMOVE             (30 * FRACUNIT)

/*- Variables ---------------------------------------------------------------*/
player_t player;
uint32_t leveltime;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void P_SpawnPlayer(void)
{
    const sector_t *sec;

    player.x = 0;
    player.y = 0;
    player.angle = 0;

    // Thing type 1 is the single player start
    for (int i = 0; i < numthings; i++)
    {
        if (1 == things[i].type)
        {
            player.x = mapfix(things[i].x);
            player.y = mapfix(things[i].y);
            player.angle = ANG45 * (things[i].angle / 45);
            break;
        }
    }

    sec = P_SectorAt(player.x, player.y);

    player.z = sec->floorheight;
    player.momx = player.momy = 0;
    player.bob = 0;

    leveltime = 0;
}

//-----------------------------------------------------------------------------
static void thrust(angle_t angle, fixed_t move)
{
    player.momx += FixedMul(move, finecosine(angle >> ANGLETOFINESHIFT));
    player.momy += FixedMul(move, finesine(angle >> ANGLETOFINESHIFT));
}

//-----------------------------------------------------------------------------
// Try the whole step, then each axis on its own, so a glancing hit slides along
// the wall instead of stopping dead
static void move_player(void)
{
    fixed_t momx = player.momx;
    fixed_t momy = player.momy;

    if (momx > MAXMOVE) momx = MAXMOVE;
    if (momx < -MAXMOVE) momx = -MAXMOVE;
    if (momy > MAXMOVE) momy = MAXMOVE;
    if (momy < -MAXMOVE) momy = -MAXMOVE;

    if (0 == momx && 0 == momy)
        return;

    if (P_TryMove(&player.x, &player.y, &player.z,
                  player.x + momx, player.y + momy,
                  PLAYER_RADIUS, PLAYER_HEIGHT))
        return;

    if (P_TryMove(&player.x, &player.y, &player.z,
                  player.x + momx, player.y,
                  PLAYER_RADIUS, PLAYER_HEIGHT))
    {
        player.momy = 0;
        return;
    }

    if (P_TryMove(&player.x, &player.y, &player.z,
                  player.x, player.y + momy,
                  PLAYER_RADIUS, PLAYER_HEIGHT))
    {
        player.momx = 0;
        return;
    }

    player.momx = player.momy = 0;
}

//-----------------------------------------------------------------------------
void P_PlayerTic(const ticcmd_t *cmd)
{
    int turn = cmd->run ? TURN_RUN : TURN_WALK;
    int forward = cmd->run ? FORWARD_RUN : FORWARD_WALK;
    int side = cmd->run ? SIDE_RUN : SIDE_WALK;

    leveltime++;

    if (cmd->turn)
        player.angle += (angle_t)(cmd->turn * turn) << 16;

    if (cmd->forward)
        thrust(player.angle, cmd->forward * forward * 2048);

    // Strafing right is angle - ANG90: angles increase counterclockwise
    if (cmd->side)
        thrust(player.angle - ANG90, cmd->side * side * 2048);

    move_player();

    // The view bob follows the speed, the way the original derives it
    player.bob = FixedMul(player.momx, player.momx) +
                 FixedMul(player.momy, player.momy);
    player.bob >>= 2;

    if (player.bob > MAXBOB)
        player.bob = MAXBOB;

    if (player.momx > -STOPSPEED && player.momx < STOPSPEED &&
        player.momy > -STOPSPEED && player.momy < STOPSPEED)
    {
        player.momx = player.momy = 0;
    }
    else
    {
        player.momx = FixedMul(player.momx, FRICTION);
        player.momy = FixedMul(player.momy, FRICTION);
    }
}

//-----------------------------------------------------------------------------
fixed_t P_ViewZ(void)
{
    // The bob is a sine of the tic count, so its period stays constant however
    // fast or slow the frames come
    unsigned angle = (FINEANGLES / 20 * leveltime) & FINEMASK;
    fixed_t z = player.z + VIEWHEIGHT + FixedMul(player.bob / 2, finesine(angle));
    const sector_t *sec = P_SectorAt(player.x, player.y);

    if (z > sec->ceilingheight - (4 * FRACUNIT))
        z = sec->ceilingheight - (4 * FRACUNIT);

    return z;
}

//-----------------------------------------------------------------------------
void P_RenderPlayerView(void)
{
    R_RenderPlayerView(player.x, player.y, P_ViewZ(), player.angle);
}
