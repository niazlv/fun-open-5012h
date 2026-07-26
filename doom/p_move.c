/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Movement and collision against the real level.
 *
 * The blockmap is a 128x128-unit grid, and each cell lists the linedefs that
 * cross it, so a move only ever tests the handful of lines near the mover. What
 * a line does to the move is decided the way the original decides it: a
 * one-sided line blocks, a two-sided one opens a gap between the higher floor
 * and the lower ceiling, and the move is refused if the gap is too short to
 * stand in or the step up is more than 24 units.
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom.h"

/*- Definitions -------------------------------------------------------------*/
#define BOXTOP              0
#define BOXBOTTOM           1
#define BOXLEFT             2
#define BOXRIGHT            3

#define MAXRADIUS           (32 * FRACUNIT)
#define MAXSTEP             (24 * FRACUNIT)

#define ST_HORIZONTAL       0
#define ST_VERTICAL         1
#define ST_POSITIVE         2
#define ST_NEGATIVE         3

/*- Variables ---------------------------------------------------------------*/
static fixed_t tmbbox[4];
static fixed_t tmz, tmheight, tmradius;
static fixed_t tmfloorz, tmceilingz, tmdropoffz;
static bool tmblocked;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline fixed_t line_dx(const maplinedef_t *ld)
{
    return mapfix(vertexes[ld->v2].x - vertexes[ld->v1].x);
}

//-----------------------------------------------------------------------------
static inline fixed_t line_dy(const maplinedef_t *ld)
{
    return mapfix(vertexes[ld->v2].y - vertexes[ld->v1].y);
}

//-----------------------------------------------------------------------------
static int line_slopetype(const maplinedef_t *ld)
{
    int dx = vertexes[ld->v2].x - vertexes[ld->v1].x;
    int dy = vertexes[ld->v2].y - vertexes[ld->v1].y;

    if (0 == dx)
        return ST_VERTICAL;

    if (0 == dy)
        return ST_HORIZONTAL;

    return ((dy ^ dx) >= 0) ? ST_POSITIVE : ST_NEGATIVE;
}

//-----------------------------------------------------------------------------
int P_PointOnLineSide(fixed_t x, fixed_t y, const maplinedef_t *ld)
{
    int ldx = vertexes[ld->v2].x - vertexes[ld->v1].x;
    int ldy = vertexes[ld->v2].y - vertexes[ld->v1].y;
    fixed_t dx, dy, left, right;

    if (0 == ldx)
        return (x <= (mapfix(vertexes[ld->v1].x))) ? (ldy > 0) : (ldy < 0);

    if (0 == ldy)
        return (y <= (mapfix(vertexes[ld->v1].y))) ? (ldx < 0) : (ldx > 0);

    dx = x - (mapfix(vertexes[ld->v1].x));
    dy = y - (mapfix(vertexes[ld->v1].y));

    left = FixedMul(ldy, dx);
    right = FixedMul(dy, ldx);

    return (right < left) ? 0 : 1;
}

//-----------------------------------------------------------------------------
// -1 when the box straddles the line, otherwise the side it is wholly on
static int P_BoxOnLineSide(const fixed_t *box, const maplinedef_t *ld)
{
    int p1, p2;

    switch (line_slopetype(ld))
    {
        case ST_HORIZONTAL:
            p1 = box[BOXTOP] > (mapfix(vertexes[ld->v1].y));
            p2 = box[BOXBOTTOM] > (mapfix(vertexes[ld->v1].y));

            if (line_dx(ld) < 0)
            {
                p1 ^= 1;
                p2 ^= 1;
            }
            break;

        case ST_VERTICAL:
            p1 = box[BOXRIGHT] < (mapfix(vertexes[ld->v1].x));
            p2 = box[BOXLEFT] < (mapfix(vertexes[ld->v1].x));

            if (line_dy(ld) < 0)
            {
                p1 ^= 1;
                p2 ^= 1;
            }
            break;

        case ST_POSITIVE:
            p1 = P_PointOnLineSide(box[BOXLEFT], box[BOXTOP], ld);
            p2 = P_PointOnLineSide(box[BOXRIGHT], box[BOXBOTTOM], ld);
            break;

        default:
            p1 = P_PointOnLineSide(box[BOXRIGHT], box[BOXTOP], ld);
            p2 = P_PointOnLineSide(box[BOXLEFT], box[BOXBOTTOM], ld);
            break;
    }

    return (p1 == p2) ? p1 : -1;
}

//-----------------------------------------------------------------------------
const sector_t *P_SectorAt(fixed_t x, fixed_t y)
{
    const mapsubsector_t *sub = &subsectors[R_PointInSubsector(x, y)];

    return seg_frontsector(&segs[sub->firstseg]);
}

//-----------------------------------------------------------------------------
// The gap a two-sided line leaves to walk through
static void P_LineOpening(const maplinedef_t *ld, fixed_t *opentop,
                          fixed_t *openbottom, fixed_t *lowfloor)
{
    const sector_t *front = &sectors[sides[ld->sidenum[0]].sector];
    const sector_t *back = &sectors[sides[ld->sidenum[1]].sector];

    *opentop = (front->ceilingheight < back->ceilingheight)
        ? front->ceilingheight : back->ceilingheight;

    if (front->floorheight > back->floorheight)
    {
        *openbottom = front->floorheight;
        *lowfloor = back->floorheight;
    }
    else
    {
        *openbottom = back->floorheight;
        *lowfloor = front->floorheight;
    }
}

//-----------------------------------------------------------------------------
static bool PIT_CheckLine(const maplinedef_t *ld)
{
    fixed_t opentop, openbottom, lowfloor;

    // Outside the line's own bounding box
    fixed_t lx1 = mapfix(vertexes[ld->v1].x), lx2 = mapfix(vertexes[ld->v2].x);
    fixed_t ly1 = mapfix(vertexes[ld->v1].y), ly2 = mapfix(vertexes[ld->v2].y);
    fixed_t lleft = lx1 < lx2 ? lx1 : lx2;
    fixed_t lright = lx1 < lx2 ? lx2 : lx1;
    fixed_t lbottom = ly1 < ly2 ? ly1 : ly2;
    fixed_t ltop = ly1 < ly2 ? ly2 : ly1;

    if (tmbbox[BOXRIGHT] <= lleft || tmbbox[BOXLEFT] >= lright ||
        tmbbox[BOXTOP] <= lbottom || tmbbox[BOXBOTTOM] >= ltop)
        return true;

    if (P_BoxOnLineSide(tmbbox, ld) != -1)
        return true;

    // A one-sided line, or one explicitly marked impassable
    if (NO_TEXTURE == ld->sidenum[1] || !(ld->flags & ML_TWOSIDED) ||
        (ld->flags & ML_BLOCKING))
    {
        tmblocked = true;
        return false;
    }

    P_LineOpening(ld, &opentop, &openbottom, &lowfloor);

    // Keep the tightest gap any crossed line leaves
    if (opentop < tmceilingz)
        tmceilingz = opentop;

    if (openbottom > tmfloorz)
        tmfloorz = openbottom;

    if (lowfloor < tmdropoffz)
        tmdropoffz = lowfloor;

    return true;
}

//-----------------------------------------------------------------------------
static bool P_BlockLinesIterator(int x, int y)
{
    int width = blockmap[2];
    int height = blockmap[3];
    const uint16_t *offsets = (const uint16_t *)(blockmap + 4);
    const int16_t *list;

    if (x < 0 || y < 0 || x >= width || y >= height)
        return true;

    // Offsets are in shorts from the start of the lump; every list starts with
    // a 0 filler and ends with -1
    list = blockmap + offsets[y * width + x];

    for (list++; *list != -1; list++)
    {
        if (!PIT_CheckLine(&lines[*list]))
            return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
bool P_CheckPosition(fixed_t x, fixed_t y, fixed_t z, fixed_t radius, fixed_t height)
{
    const sector_t *sec;
    int xl, xh, yl, yh;
    fixed_t bmorgx = mapfix(blockmap[0]);
    fixed_t bmorgy = mapfix(blockmap[1]);

    tmz = z;
    tmradius = radius;
    tmheight = height;
    tmblocked = false;

    tmbbox[BOXTOP] = y + radius;
    tmbbox[BOXBOTTOM] = y - radius;
    tmbbox[BOXRIGHT] = x + radius;
    tmbbox[BOXLEFT] = x - radius;

    // The subsector the centre lands in gives the starting floor and ceiling;
    // crossed lines can only tighten them
    sec = P_SectorAt(x, y);
    tmfloorz = tmdropoffz = sec->floorheight;
    tmceilingz = sec->ceilingheight;

    xl = (tmbbox[BOXLEFT] - bmorgx - MAXRADIUS) >> (FRACBITS + 7);
    xh = (tmbbox[BOXRIGHT] - bmorgx + MAXRADIUS) >> (FRACBITS + 7);
    yl = (tmbbox[BOXBOTTOM] - bmorgy - MAXRADIUS) >> (FRACBITS + 7);
    yh = (tmbbox[BOXTOP] - bmorgy + MAXRADIUS) >> (FRACBITS + 7);

    for (int by = yl; by <= yh; by++)
    {
        for (int bx = xl; bx <= xh; bx++)
        {
            if (!P_BlockLinesIterator(bx, by))
                return false;
        }
    }

    return !tmblocked;
}

//-----------------------------------------------------------------------------
// Move if the destination is reachable, and take the new floor with it
bool P_TryMove(fixed_t *x, fixed_t *y, fixed_t *z, fixed_t nx, fixed_t ny,
               fixed_t radius, fixed_t height)
{
    if (!P_CheckPosition(nx, ny, *z, radius, height))
        return false;

    if (tmceilingz - tmfloorz < height)
        return false;                       // will not fit

    if (tmceilingz - *z < height)
        return false;                       // head would hit the ceiling

    if (tmfloorz - *z > MAXSTEP)
        return false;                       // step is too high to climb

    *x = nx;
    *y = ny;
    *z = tmfloorz;

    return true;
}

//-----------------------------------------------------------------------------
fixed_t P_FloorHeight(fixed_t x, fixed_t y)
{
    return P_SectorAt(x, y)->floorheight;
}
