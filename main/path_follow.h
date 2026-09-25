#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "nav_geo.h"

/* Follow a path in local metres -- the Guidance layer (spec section 3).  The
 * planner hands it a short polyline; it steers along the ACTIVE segment with
 * lookahead LOS (Breivik & Fossen) and moves to the next segment at that
 * segment's finish line.  Today every path is one straight segment
 * (planner.c).  A waypoint mission, or a SUSHI-style planner bending the
 * route around obstacles, simply hands it more points -- nothing downstream
 * changes.  Pure: no hardware, host-tested. */

#define NAV_PATH_MAX_POINTS 16

typedef struct {
    nav_en_t pts[NAV_PATH_MAX_POINTS];
    uint8_t n;                    /* >= 2 for a usable path */
} nav_path_t;

typedef struct {
    uint8_t seg;                  /* active segment: pts[seg] -> pts[seg + 1] */
} path_follow_t;

typedef struct {
    float course_deg;             /* wanted course over ground */
    nav_line_pos_t line;          /* the boat against the ACTIVE segment */
    float bearing_to_end_deg;     /* straight at the path's last point */
    bool last_segment;
    bool past_end;                /* beyond the last segment's finish line */
} path_follow_out_t;

void path_follow_reset(path_follow_t *f);

/* One step at a new position.  A segment is left only at its finish line
 * (never re-entered), and never beyond the last one.  Past the end without
 * having arrived -- only possible far off the line -- the course points at
 * the end, so the boat turns back rather than carrying on. */
path_follow_out_t path_follow_step(path_follow_t *f, const nav_path_t *path, nav_en_t pos,
                                   float lookahead_m);
