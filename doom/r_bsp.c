/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * BSP traversal and the solid segment clip list.
 *
 * The tree is walked front to back, and every wall that turns out to be solid
 * is inserted into solidsegs - a sorted list of screen column ranges that are
 * already fully covered. A subtree whose bounding box projects entirely inside
 * one of those ranges is skipped without ever being visited. That is what makes
 * the renderer's cost track what is visible instead of what exists, which is
 * the only reason a level this size renders at all on this part.
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom.h"

/*- Definitions -------------------------------------------------------------*/
#define NF_SUBSECTOR        0x8000

#define BOXTOP              0
#define BOXBOTTOM           1
#define BOXLEFT             2
#define BOXRIGHT            3

/*- Variables ---------------------------------------------------------------*/
const mapseg_t *curline;
const mapside_t *sidedef;
const maplinedef_t *linedef;
const sector_t *frontsector;
const sector_t *backsector;
angle_t rw_angle1;

drawseg_t *ds_p;

static cliprange_t *newend;

// Which corners of a bounding box form its silhouette, indexed by where the
// viewer sits relative to the box
static const uint8_t checkcoord[12][4] =
{
    {3, 0, 2, 1},
    {3, 0, 2, 0},
    {3, 1, 2, 0},
    {0, 0, 0, 0},
    {2, 0, 2, 1},
    {0, 0, 0, 0},
    {3, 1, 3, 0},
    {0, 0, 0, 0},
    {2, 0, 3, 1},
    {2, 1, 3, 1},
    {2, 1, 3, 0},
    {0, 0, 0, 0},
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void R_ClearDrawSegs(void)
{
    ds_p = dm->drawsegs;
}

//-----------------------------------------------------------------------------
void R_ClearClipSegs(void)
{
    dm->solidsegs[0].first = -0x7fffffff;
    dm->solidsegs[0].last = -1;
    dm->solidsegs[1].first = SCREENWIDTH;
    dm->solidsegs[1].last = 0x7fffffff;
    newend = dm->solidsegs + 2;
}

//-----------------------------------------------------------------------------
// A solid wall: draw the parts of it that are not covered yet, then merge its
// range into the clip list
static void R_ClipSolidWallSegment(int first, int last)
{
    cliprange_t *next, *start;

    start = dm->solidsegs;

    while (start->last < first - 1)
        start++;

    if (first < start->first)
    {
        if (last < start->first - 1)
        {
            // Entirely visible, insert a new range
            R_StoreWallRange(first, last);

            // Out of clip ranges: dropping the merge would corrupt the list,
            // so leave the wall drawn but unrecorded
            if (newend >= dm->solidsegs + MAXSEGS)
                return;

            next = newend;
            newend++;

            while (next != start)
            {
                *next = *(next - 1);
                next--;
            }

            next->first = first;
            next->last = last;

            if (newend - dm->solidsegs > r_stats.solidsegs)
                r_stats.solidsegs = newend - dm->solidsegs;

            return;
        }

        // A fragment above the range
        R_StoreWallRange(first, start->first - 1);
        start->first = first;
    }

    if (last <= start->last)
        return;

    next = start;

    while (last >= (next + 1)->first - 1)
    {
        // A fragment between two ranges
        R_StoreWallRange(next->last + 1, (next + 1)->first - 1);
        next++;

        if (last <= next->last)
        {
            start->last = next->last;
            goto crunch;
        }
    }

    R_StoreWallRange(next->last + 1, last);
    start->last = last;

crunch:
    if (next == start)
        return;

    // start now spans everything up to next, so drop what is in between
    while (next++ != newend)
        *++start = *next;

    newend = start + 1;
}

//-----------------------------------------------------------------------------
// A window: draw the uncovered parts but do not close anything off
static void R_ClipPassWallSegment(int first, int last)
{
    cliprange_t *start;

    start = dm->solidsegs;

    while (start->last < first - 1)
        start++;

    if (first < start->first)
    {
        if (last < start->first - 1)
        {
            R_StoreWallRange(first, last);
            return;
        }

        R_StoreWallRange(first, start->first - 1);
    }

    if (last <= start->last)
        return;

    while (last >= (start + 1)->first - 1)
    {
        R_StoreWallRange(start->last + 1, (start + 1)->first - 1);
        start++;

        if (last <= start->last)
            return;
    }

    R_StoreWallRange(start->last + 1, last);
}

//-----------------------------------------------------------------------------
static inline int angle_to_x(angle_t a)
{
    unsigned i = (a + ANG90) >> ANGLETOFINESHIFT;

    // The clipped angle can land exactly one past the end of the table
    if (i > 4095)
        i = 4095;

    return dt_viewangletox[i];
}

//-----------------------------------------------------------------------------
static void R_AddLine(const mapseg_t *line)
{
    int x1, x2;
    angle_t angle1, angle2, span, tspan;

    curline = line;

    angle1 = R_PointToAngle(mapfix(vertexes[line->v1].x),
                            mapfix(vertexes[line->v1].y));
    angle2 = R_PointToAngle(mapfix(vertexes[line->v2].x),
                            mapfix(vertexes[line->v2].y));

    // Back side of the wall: the endpoints run the other way around
    span = angle1 - angle2;

    if (span >= ANG180)
        return;

    rw_angle1 = angle1;
    angle1 -= viewangle;
    angle2 -= viewangle;

    tspan = angle1 + clipangle;

    if (tspan > 2 * clipangle)
    {
        tspan -= 2 * clipangle;

        if (tspan >= span)
            return;                 // entirely off the left edge

        angle1 = clipangle;
    }

    tspan = clipangle - angle2;

    if (tspan > 2 * clipangle)
    {
        tspan -= 2 * clipangle;

        if (tspan >= span)
            return;                 // entirely off the right edge

        angle2 = (angle_t)(-(int32_t)clipangle);
    }

    x1 = angle_to_x(angle1);
    x2 = angle_to_x(angle2);

    if (x1 == x2)
        return;                     // does not cross a pixel centre

    linedef = &lines[line->linedef];
    sidedef = seg_sidedef(line);
    backsector = seg_backsector(line);

    if (NULL == backsector)
        goto clipsolid;

    // A closed door, or a sector shut tight: solid from here
    if (backsector->ceilingheight <= frontsector->floorheight ||
        backsector->floorheight >= frontsector->ceilingheight)
        goto clipsolid;

    // A step or a window: see through it, but its edges still draw
    if (backsector->ceilingheight != frontsector->ceilingheight ||
        backsector->floorheight != frontsector->floorheight)
        goto clippass;

    // Nothing to draw at all - these are the lines that only carry triggers
    if (backsector->ceilingpic == frontsector->ceilingpic &&
        backsector->floorpic == frontsector->floorpic &&
        backsector->lightlevel == frontsector->lightlevel &&
        NO_TEXTURE == sidedef->midtex)
        return;

clippass:
    R_ClipPassWallSegment(x1, x2 - 1);
    return;

clipsolid:
    R_ClipSolidWallSegment(x1, x2 - 1);
}

//-----------------------------------------------------------------------------
// Is any part of this subtree's bounding box in a column that is still open?
static bool R_CheckBBox(const int16_t *bspcoord)
{
    int boxx, boxy, boxpos;
    fixed_t x1, y1, x2, y2;
    angle_t angle1, angle2, span, tspan;
    const cliprange_t *start;
    int sx1, sx2;
    fixed_t box[4];

    box[0] = mapfix(bspcoord[0]);
    box[1] = mapfix(bspcoord[1]);
    box[2] = mapfix(bspcoord[2]);
    box[3] = mapfix(bspcoord[3]);

    if (viewx <= box[BOXLEFT])
        boxx = 0;
    else if (viewx < box[BOXRIGHT])
        boxx = 1;
    else
        boxx = 2;

    if (viewy >= box[BOXTOP])
        boxy = 0;
    else if (viewy > box[BOXBOTTOM])
        boxy = 1;
    else
        boxy = 2;

    boxpos = (boxy << 2) + boxx;

    if (5 == boxpos)
        return true;                // inside the box

    x1 = box[checkcoord[boxpos][0]];
    y1 = box[checkcoord[boxpos][1]];
    x2 = box[checkcoord[boxpos][2]];
    y2 = box[checkcoord[boxpos][3]];

    angle1 = R_PointToAngle(x1, y1) - viewangle;
    angle2 = R_PointToAngle(x2, y2) - viewangle;

    span = angle1 - angle2;

    if (span >= ANG180)
        return true;                // sitting on the line

    tspan = angle1 + clipangle;

    if (tspan > 2 * clipangle)
    {
        tspan -= 2 * clipangle;

        if (tspan >= span)
            return false;

        angle1 = clipangle;
    }

    tspan = clipangle - angle2;

    if (tspan > 2 * clipangle)
    {
        tspan -= 2 * clipangle;

        if (tspan >= span)
            return false;

        angle2 = (angle_t)(-(int32_t)clipangle);
    }

    sx1 = angle_to_x(angle1);
    sx2 = angle_to_x(angle2);

    if (sx1 == sx2)
        return false;

    sx2--;

    start = dm->solidsegs;

    while (start->last < sx2)
        start++;

    // Fully inside a range that is already covered
    if (sx1 >= start->first && sx2 <= start->last)
        return false;

    return true;
}

//-----------------------------------------------------------------------------
// The subsector's sector is not stored; it is the sector of the first seg's
// sidedef, which is how the node builder defines it
static const sector_t *subsector_sector(const mapsubsector_t *sub)
{
    return seg_frontsector(&segs[sub->firstseg]);
}

//-----------------------------------------------------------------------------
static void R_Subsector(int num)
{
    const mapsubsector_t *sub = &subsectors[num];
    const mapseg_t *line = &segs[sub->firstseg];
    int count = sub->numsegs;

    frontsector = subsector_sector(sub);

    floorplane = (frontsector->floorheight < viewz)
        ? R_FindPlane(frontsector->floorheight, frontsector->floorpic,
                      frontsector->lightlevel)
        : NULL;

    // The sky is drawn even when the ceiling is below the eye, which is what
    // keeps it visible over low walls
    ceilingplane = (frontsector->ceilingheight > viewz ||
                    SKY_FLAT == frontsector->ceilingpic)
        ? R_FindPlane(frontsector->ceilingheight, frontsector->ceilingpic,
                      frontsector->lightlevel)
        : NULL;

    while (count--)
        R_AddLine(line++);
}

//-----------------------------------------------------------------------------
void R_RenderBSPNode(int bspnum)
{
    const mapnode_t *bsp;
    int side;

    // Near side first, always - the clip list only closes columns correctly if
    // the tree is walked front to back. The far side is the tail call, so it
    // becomes the loop and the recursion depth halves.
    while (!(bspnum & NF_SUBSECTOR))
    {
        bsp = &nodes[bspnum];
        side = R_PointOnSide(viewx, viewy, bsp);

        R_RenderBSPNode(bsp->children[side]);

        if (!R_CheckBBox(bsp->bbox[side ^ 1]))
            return;

        bspnum = bsp->children[side ^ 1];
    }

    R_Subsector((-1 == bspnum) ? 0 : (bspnum & ~NF_SUBSECTOR));
}
