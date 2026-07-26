/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Host harness for the DOOM renderer.
 *
 * The renderer has no hardware dependencies - it writes 8-bit palette indices
 * into a block of memory - so the same sources that go on the device can be
 * built and run here, and the frame dumped as an image. Catching a broken angle
 * table or an inverted silhouette takes a second here and a reflash there.
 *
 *   cc -O2 -I doom -o /tmp/doomhost tests/doom_host.c doom/*.c
 *   /tmp/doomhost doom/doom_assets.bin out.ppm [x y angle]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "doom.h"

#define VIEWHEIGHT      (41 * FRACUNIT)

static void *load_file(const char *path, long *size)
{
    FILE *f = fopen(path, "rb");
    void *buf;

    if (!f)
    {
        perror(path);
        exit(1);
    }

    fseek(f, 0, SEEK_END);
    *size = ftell(f);
    fseek(f, 0, SEEK_SET);

    buf = malloc(*size);

    if (1 != fread(buf, *size, 1, f))
    {
        fprintf(stderr, "short read on %s\n", path);
        exit(1);
    }

    fclose(f);

    return buf;
}

static void write_ppm(const char *path)
{
    FILE *f = fopen(path, "wb");

    fprintf(f, "P6\n%d %d\n255\n", SCREENWIDTH, SCREENHEIGHT);

    for (int i = 0; i < SCREENWIDTH * SCREENHEIGHT; i++)
    {
        // The pack stores the palette already converted to the display's
        // RGB565, so expand it back for the dump
        uint16_t c = dt_palette[dm->screen[i]];
        unsigned char rgb[3];

        rgb[0] = ((c >> 11) & 0x1f) * 255 / 31;
        rgb[1] = ((c >> 5) & 0x3f) * 255 / 63;
        rgb[2] = (c & 0x1f) * 255 / 31;

        fwrite(rgb, 3, 1, f);
    }

    fclose(f);
}

int main(int argc, char **argv)
{
    long size;
    void *blob;
    fixed_t x = 0, y = 0, z;
    angle_t angle = 0;
    const sector_t *sec;
    bool found = false;

    if (argc < 3)
    {
        fprintf(stderr, "usage: %s assets.bin out.ppm [x y angle_deg]\n", argv[0]);
        return 1;
    }

    blob = load_file(argv[1], &size);

    dm = calloc(1, sizeof(doom_mem_t));
    printf("doom_mem_t: %zu bytes (%.1f KB of the 128 KB block)\n",
           sizeof(doom_mem_t), sizeof(doom_mem_t) / 1024.0);

    if (!doom_assets_init(blob))
    {
        fprintf(stderr, "asset pack rejected\n");
        return 1;
    }

    if (!doom_level_load())
    {
        fprintf(stderr, "level load failed\n");
        return 1;
    }

    printf("map %s: %d segs, %d subsectors, %d nodes, %d sectors, %d textures\n",
           doom_level_name(), numsegs, numsubsectors, numnodes, numsectors,
           numtextures);

    R_Init();
    R_InitSegs();

    // Thing type 1 is the single player start
    for (int i = 0; i < numthings; i++)
    {
        if (1 == things[i].type)
        {
            x = mapfix(things[i].x);
            y = mapfix(things[i].y);
            angle = ANG45 * (things[i].angle / 45);
            found = true;
            break;
        }
    }

    if (!found)
    {
        fprintf(stderr, "no player start in this map\n");
        return 1;
    }

    // Stand in front of every door line in the map, press use, and watch what
    // the ceiling does. This exercises both the ray that finds the line and the
    // thinker that moves it.
    if (0 == strcmp(argv[2], "--doors"))
    {
        int found = 0, opened = 0;

        for (int i = 0; i < numlines; i++)
        {
            const maplinedef_t *ld = &lines[i];
            fixed_t mx, my, nx, ny, len;
            fixed_t before, after;
            int sector;
            angle_t face;

            switch (ld->special)
            {
                case 1: case 26: case 27: case 28: case 31: case 32:
                case 33: case 34: case 117: case 118:
                    break;
                default:
                    continue;
            }

            if (NO_TEXTURE == ld->sidenum[1])
                continue;

            found++;

            // Stand 40 units off the middle of the line, on its front side,
            // looking at it
            mx = (mapfix(vertexes[ld->v1].x) + mapfix(vertexes[ld->v2].x)) / 2;
            my = (mapfix(vertexes[ld->v1].y) + mapfix(vertexes[ld->v2].y)) / 2;

            nx = mapfix(vertexes[ld->v2].y) - mapfix(vertexes[ld->v1].y);
            ny = mapfix(vertexes[ld->v1].x) - mapfix(vertexes[ld->v2].x);
            {
                fixed_t sx = viewx, sy = viewy;

                viewx = 0;
                viewy = 0;
                len = R_PointToDist(nx, ny);
                viewx = sx;
                viewy = sy;
            }

            if (0 == len)
                continue;

            nx = FixedDiv(nx, len);
            ny = FixedDiv(ny, len);

            // Each door has two lines; reload so every one is tested
            // against a closed door rather than the previous result
            doom_level_load();
            P_DoorsClear();

            player.x = mx + FixedMul(nx, 40 * FRACUNIT);
            player.y = my + FixedMul(ny, 40 * FRACUNIT);
            player.z = P_SectorAt(player.x, player.y)->floorheight;
            face = R_PointToAngle2(player.x, player.y, mx, my);

            sector = sides[ld->sidenum[
                P_PointOnLineSide(player.x, player.y, ld) ^ 1]].sector;
            before = sectors[sector].ceilingheight;

            if (!P_UseLines(player.x, player.y, face))
            {
                printf("  line %3d special %3d: use found nothing\n",
                       i, ld->special);
                continue;
            }

            for (int t = 0; t < 120; t++)
                P_DoorsTic(player.x, player.y, player.z);

            after = sectors[sector].ceilingheight;

            printf("  line %3d special %3d: sector %3d ceiling %4ld -> %4ld  %s\n",
                   i, ld->special, sector, (long)(before >> FRACBITS),
                   (long)(after >> FRACBITS), after > before ? "OPENED" : "stuck");

            if (after > before)
                opened++;
        }

        printf("%d door lines, %d opened\n", found, opened);

        return 0;
    }

    // Play a scripted route through the level and dump every Nth frame, so the
    // result can be watched without a device. The simulation and the renderer
    // are the ones the firmware runs - only the input source differs.
    if (0 == strcmp(argv[2], "--demo"))
    {
        static const struct { int tics; int8_t forward, side, turn; bool run; } route[] =
        {
            // Turning is 3.5 degrees per tic walking, 7 running, and walking
            // covers about 8 units per tic, which is what the lengths below are
            // sized from. The route stays inside the first room.
            { 25,  0,  0,  0, false },      // the wall we spawn facing
            { 62,  1,  0,  0, false },      // north, out of the alcove
            { 26,  0,  0, -1, false },      // pan right to face east
            { 22,  1,  0,  0, false },      // east across the room
            { 26,  0,  0,  1, false },      // face north
            { 26,  1,  0,  0, false },      // up towards the window wall
            { 26,  0,  0,  1, false },      // face west, along the windows
            { 30,  1,  0,  0, true  },      // run west past them
            { 24,  0,  0,  1, false },      // turn south, back into the room
            { 22,  1,  0,  0, false },
            { 26,  0, -1,  0, false },      // strafe left
            { 20,  0,  0,  0, false },
        };

        const int every = 3;
        FILE *f = fopen("/tmp/doom_demo.raw", "wb");
        uint8_t rgb[768];
        int total = 0, frames = 0;

        P_SpawnPlayer();

        for (int i = 0; i < (int)(sizeof(route) / sizeof(route[0])); i++)
            total += route[i].tics;

        frames = 0;

        for (int i = 0; i < 256; i++)
        {
            uint16_t c = dt_palette[i];

            rgb[i * 3 + 0] = ((c >> 11) & 0x1f) * 255 / 31;
            rgb[i * 3 + 1] = ((c >> 5) & 0x3f) * 255 / 63;
            rgb[i * 3 + 2] = (c & 0x1f) * 255 / 31;
        }

        // Header is patched with the real frame count once they are all written
        fwrite(&frames, 4, 1, f);
        fwrite(rgb, sizeof(rgb), 1, f);

        for (int i = 0; i < (int)(sizeof(route) / sizeof(route[0])); i++)
        {
            ticcmd_t cmd = { route[i].forward, route[i].side, route[i].turn,
                             route[i].run };

            for (int t = 0; t < route[i].tics; t++)
            {
                P_PlayerTic(&cmd);

                if (0 == (leveltime % every))
                {
                    P_RenderPlayerView();
                    fwrite(dm->screen, SCREENWIDTH * SCREENHEIGHT, 1, f);
                    frames++;
                }
            }

            printf("  step %2d: (%6ld, %6ld) facing %3ld deg\n", i,
                   (long)(player.x >> FRACBITS), (long)(player.y >> FRACBITS),
                   (long)((uint64_t)player.angle * 360 >> 32));
        }

        fseek(f, 0, SEEK_SET);
        fwrite(&frames, 4, 1, f);
        fclose(f);

        printf("%d tics, %d frames -> /tmp/doom_demo.raw\n", total, frames);
        printf("ended at (%ld, %ld)\n", (long)(player.x >> FRACBITS),
               (long)(player.y >> FRACBITS));

        return 0;
    }

    // Walk into the level from the player start in each of eight directions
    // and check the collision code stops the player at a wall rather than
    // letting them leave the map or jamming them in place
    if (0 == strcmp(argv[2], "--walk"))
    {
        const fixed_t radius = 16 * FRACUNIT, height = 56 * FRACUNIT;
        const fixed_t step = 8 * FRACUNIT;
        fixed_t sx = x, sy = y;

        for (int a = 0; a < 8; a++)
        {
            angle_t dir = a * ANG45;
            fixed_t px = sx, py = sy, pz;
            int moved = 0;

            pz = P_SectorAt(px, py)->floorheight;

            for (int i = 0; i < 200; i++)
            {
                fixed_t nx = px + FixedMul(step, finecosine(dir >> ANGLETOFINESHIFT));
                fixed_t ny = py + FixedMul(step, finesine(dir >> ANGLETOFINESHIFT));

                if (!P_TryMove(&px, &py, &pz, nx, ny, radius, height))
                    break;

                moved++;
            }

            printf("  %3d deg: %3d steps -> (%6ld, %6ld) floor %4ld\n",
                   a * 45, moved, (long)(px >> FRACBITS), (long)(py >> FRACBITS),
                   (long)(pz >> FRACBITS));
        }

        return 0;
    }

    // Walk every subsector from eight angles and report how close the frame
    // came to the fixed limits. This is what the array sizes are chosen from.
    if (0 == strcmp(argv[2], "--sweep"))
    {
        int frames = 0;

        for (int i = 0; i < numsubsectors; i++)
        {
            const mapsubsector_t *sub = &subsectors[i];
            const sector_t *s;
            long sx = 0, sy = 0;

            for (int k = 0; k < sub->numsegs; k++)
            {
                sx += vertexes[segs[sub->firstseg + k].v1].x;
                sy += vertexes[segs[sub->firstseg + k].v1].y;
            }

            if (0 == sub->numsegs)
                continue;

            // The average of the subsector's vertices is inside its convex hull
            x = mapfix((int)(sx / sub->numsegs));
            y = mapfix((int)(sy / sub->numsegs));

            s = P_SectorAt(x, y);
            z = s->floorheight + VIEWHEIGHT;

            if (z > s->ceilingheight - (4 << FRACBITS))
                z = s->ceilingheight - (4 << FRACBITS);

            for (int a = 0; a < 8; a++)
            {
                R_RenderPlayerView(x, y, z, (angle_t)(a * (ANG45)));
                frames++;
            }
        }

        printf("swept %d views over %d subsectors\n", frames, numsubsectors);
        printf("  visplanes %3d / %d\n", r_stats.visplanes, MAXVISPLANES);
        printf("  drawsegs  %3d / %d\n", r_stats.drawsegs, MAXDRAWSEGS);
        printf("  openings  %3d / %d\n", r_stats.openings, MAXOPENINGS);
        printf("  solidsegs %3d / %d\n", r_stats.solidsegs, MAXSEGS);
        printf("  masked columns drawn %d\n", r_stats.maskedcols);

        return 0;
    }

    if (argc >= 6)
    {
        x = mapfix(atoi(argv[3]));
        y = mapfix(atoi(argv[4]));
        angle = (angle_t)((double)atof(argv[5]) / 360.0 * 4294967296.0);
    }

    sec = &sectors[sides[lines[segs[subsectors[R_PointInSubsector(x, y)].firstseg]
        .linedef].sidenum[segs[subsectors[R_PointInSubsector(x, y)].firstseg].side]].sector];
    z = sec->floorheight + VIEWHEIGHT;

    printf("view at (%d, %d, %d) angle %.1f deg\n", x >> FRACBITS, y >> FRACBITS,
           z >> FRACBITS, angle / 4294967296.0 * 360.0);

    R_RenderPlayerView(x, y, z, angle);

    write_ppm(argv[2]);
    printf("wrote %s\n", argv[2]);

    return 0;
}
