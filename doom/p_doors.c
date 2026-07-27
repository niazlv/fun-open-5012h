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
 * Doors.
 *
 * A door is a sector whose ceiling moves: it rises to just under the lowest
 * ceiling around it, waits, and comes back down. Nothing else in the engine
 * needs to know - the renderer reads sector heights every frame and the
 * collision code derives the gap a two-sided line leaves from the same two
 * numbers, so a moving ceiling is drawn and walked through correctly for free.
 *
 * Pressing use casts a short ray and activates the nearest special line it
 * crosses, which is how the original finds what the player is standing in front
 * of. Locked doors open regardless here: there is no inventory yet, and a door
 * that can never open is worse than one that opens early.
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom.h"

/*- Definitions -------------------------------------------------------------*/
#define MAXDOORS            12
#define VDOORSPEED          (FRACUNIT * 2)
#define VDOORWAIT           150
#define USERANGE            (64 * FRACUNIT)

// Door line specials, by what they do rather than by number
#define D_NORMAL            0       // open, wait, close - can be used again
#define D_OPEN              1       // open and stay open

/*- Types -------------------------------------------------------------------*/
typedef enum
{
    DOOR_FREE = 0,
    DOOR_OPENING,
    DOOR_WAITING,
    DOOR_CLOSING,
} doorstate_t;

typedef struct
{
    int16_t sector;
    uint8_t state;
    uint8_t kind;
    fixed_t topheight;
    fixed_t speed;
    int16_t counter;
} door_t;

/*- Variables ---------------------------------------------------------------*/
static door_t g_doors[MAXDOORS];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void P_DoorsClear(void)
{
    for (int i = 0; i < MAXDOORS; i++)
        g_doors[i].state = DOOR_FREE;
}

//-----------------------------------------------------------------------------
static door_t *door_for_sector(int sector)
{
    for (int i = 0; i < MAXDOORS; i++)
    {
        if (DOOR_FREE != g_doors[i].state && g_doors[i].sector == sector)
            return &g_doors[i];
    }

    return NULL;
}

//-----------------------------------------------------------------------------
// How high the door can go: the lowest ceiling of the sectors around it, less
// the four units the original leaves so the door's own top texture still shows
static fixed_t lowest_ceiling_around(int sector)
{
    fixed_t lowest = INT32_MAX;

    for (int i = 0; i < numlines; i++)
    {
        const maplinedef_t *ld = &lines[i];
        int front, back;

        if (NO_TEXTURE == ld->sidenum[0] || NO_TEXTURE == ld->sidenum[1])
            continue;

        front = sides[ld->sidenum[0]].sector;
        back = sides[ld->sidenum[1]].sector;

        if (front == sector)
        {
            if (sectors[back].ceilingheight < lowest)
                lowest = sectors[back].ceilingheight;
        }
        else if (back == sector)
        {
            if (sectors[front].ceilingheight < lowest)
                lowest = sectors[front].ceilingheight;
        }
    }

    if (INT32_MAX == lowest)
        lowest = sectors[sector].ceilingheight;

    return lowest - 4 * FRACUNIT;
}

//-----------------------------------------------------------------------------
static bool activate_door(int sector, int kind)
{
    door_t *door = door_for_sector(sector);
    int slot;

    if (door)
    {
        // Using a door that is already moving reverses it, which is what makes
        // a door you walked into come back up
        if (DOOR_CLOSING == door->state)
            door->state = DOOR_OPENING;
        else if (DOOR_OPENING == door->state)
            door->state = DOOR_CLOSING;
        else
            door->counter = 0;      // waiting: close it now

        return true;
    }

    for (slot = 0; slot < MAXDOORS; slot++)
    {
        if (DOOR_FREE == g_doors[slot].state)
            break;
    }

    if (MAXDOORS == slot)
        return false;

    door = &g_doors[slot];

    door->sector = sector;
    door->kind = kind;
    door->speed = VDOORSPEED;
    door->topheight = lowest_ceiling_around(sector);
    door->counter = VDOORWAIT;
    door->state = DOOR_OPENING;

    return true;
}

//-----------------------------------------------------------------------------
static int door_kind(int special)
{
    switch (special)
    {
        case 1: case 26: case 27: case 28:      // DR, and the keyed variants
        case 117:                               // blazing
            return D_NORMAL;

        case 31: case 32: case 33: case 34:     // D1, open and stay
        case 118:
            return D_OPEN;

        default:
            return -1;
    }
}

//-----------------------------------------------------------------------------
void P_DoorsTic(fixed_t playerx, fixed_t playery, fixed_t playerz)
{
    for (int i = 0; i < MAXDOORS; i++)
    {
        door_t *door = &g_doors[i];
        sector_t *sec;

        if (DOOR_FREE == door->state)
            continue;

        sec = &sectors[door->sector];

        if (DOOR_OPENING == door->state)
        {
            sec->ceilingheight += door->speed;

            if (sec->ceilingheight >= door->topheight)
            {
                sec->ceilingheight = door->topheight;

                if (D_OPEN == door->kind)
                    door->state = DOOR_FREE;
                else
                {
                    door->state = DOOR_WAITING;
                    door->counter = VDOORWAIT;
                }
            }
        }
        else if (DOOR_WAITING == door->state)
        {
            if (--door->counter <= 0)
                door->state = DOOR_CLOSING;
        }
        else
        {
            // Do not close on the player's head: reverse instead, the way the
            // original's crush check does
            if (playerz + PLAYER_HEIGHT > sec->ceilingheight - door->speed &&
                &sectors[door->sector] == P_SectorAt(playerx, playery))
            {
                door->state = DOOR_OPENING;
                continue;
            }

            sec->ceilingheight -= door->speed;

            if (sec->ceilingheight <= sec->floorheight)
            {
                sec->ceilingheight = sec->floorheight;
                door->state = DOOR_FREE;
            }
        }
    }
}

//-----------------------------------------------------------------------------
static int64_t cross(fixed_t ax, fixed_t ay, fixed_t bx, fixed_t by)
{
    return (int64_t)ax * by - (int64_t)ay * bx;
}

//-----------------------------------------------------------------------------
// Nearest special line crossed by a short ray in front of the player
bool P_UseLines(fixed_t x, fixed_t y, angle_t angle)
{
    fixed_t dx = FixedMul(USERANGE, finecosine(angle >> ANGLETOFINESHIFT));
    fixed_t dy = FixedMul(USERANGE, finesine(angle >> ANGLETOFINESHIFT));
    fixed_t bestfrac = FRACUNIT + 1;
    int best = -1;

    for (int i = 0; i < numlines; i++)
    {
        const maplinedef_t *ld = &lines[i];
        fixed_t lx = mapfix(vertexes[ld->v1].x);
        fixed_t ly = mapfix(vertexes[ld->v1].y);
        fixed_t ldx = mapfix(vertexes[ld->v2].x) - lx;
        fixed_t ldy = mapfix(vertexes[ld->v2].y) - ly;
        int64_t d1, d2, e1, e2, a, b;
        fixed_t frac;

        if (0 == ld->special)
            continue;

        // Do the two ends of the line straddle the ray, and the two ends of the
        // ray straddle the line? Then they cross.
        d1 = cross(dx, dy, lx - x, ly - y);
        d2 = cross(dx, dy, lx + ldx - x, ly + ldy - y);

        if ((d1 < 0) == (d2 < 0))
            continue;

        e1 = cross(ldx, ldy, x - lx, y - ly);
        e2 = cross(ldx, ldy, x + dx - lx, y + dy - ly);

        if ((e1 < 0) == (e2 < 0))
            continue;

        // How far along the ray, in 16.16. The shift keeps the division inside
        // 64 bits for map-sized coordinates.
        a = e1 >> 16;
        b = (e1 - e2) >> 16;

        if (0 == b)
            continue;

        frac = (fixed_t)((a * FRACUNIT) / b);

        if (frac >= 0 && frac < bestfrac)
        {
            bestfrac = frac;
            best = i;
        }
    }

    if (best < 0)
        return false;

    {
        const maplinedef_t *ld = &lines[best];
        int kind = door_kind(ld->special);
        int side;
        uint16_t sn;

        if (kind < 0)
            return false;

        // The door is the sector on the far side of the line from the player,
        // which is not always the back side - a door can be used from either
        side = P_PointOnLineSide(x, y, ld);
        sn = ld->sidenum[side ^ 1];

        if (NO_TEXTURE == sn)
            return false;

        return activate_door(sides[sn].sector, kind);
    }
}
