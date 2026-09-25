#pragma once

#include <stdbool.h>

/* Flat-earth maths for short missions (tens to a few hundred metres).  An
 * equirectangular projection about a fixed origin: its error is far below GPS
 * noise at these ranges.  Angles are compass degrees -- 0 = north, 90 = east,
 * clockwise.  No ESP-IDF dependencies. */

typedef struct {
    double lat0_deg;
    double lon0_deg;
    double m_per_deg_lat;
    double m_per_deg_lon;
} nav_origin_t;

typedef struct {
    float e;   /* metres east of the origin */
    float n;   /* metres north of the origin */
} nav_en_t;

/* Where a point sits relative to the line a -> b. */
typedef struct {
    float along_m;    /* metres along the line from a (negative: behind a) */
    float cross_m;    /* metres to the RIGHT of the line, looking from a to b */
    float progress;   /* along_m / length: 1.0 = level with b ("finish line") */
    float length_m;
} nav_line_pos_t;

void nav_origin_init(nav_origin_t *o, double lat_deg, double lon_deg);
nav_en_t nav_to_local(const nav_origin_t *o, double lat_deg, double lon_deg);
void nav_to_latlon(const nav_origin_t *o, nav_en_t p, double *lat_deg, double *lon_deg);

float nav_wrap_360(float deg);
float nav_wrap_180(float deg);
float nav_distance_m(nav_en_t a, nav_en_t b);
/* Compass bearing from `from` to `to`, 0..360; 0 when they coincide. */
float nav_bearing_deg(nav_en_t from, nav_en_t to);
/* The point `dist_m` from `p` along compass bearing `bearing_deg`. */
nav_en_t nav_offset(nav_en_t p, float bearing_deg, float dist_m);

/* ArduPilot's Location::line_path_proportion, plus the cross-track error.  A
 * degenerate line (a and b within 3 cm) reports progress 1: already there. */
nav_line_pos_t nav_line_position(nav_en_t a, nav_en_t b, nav_en_t p);

/* Lookahead line-of-sight course (Breivik & Fossen 2009): the compass course
 * that steers onto the line a -> b, aiming `lookahead_m` ahead along it:
 *     course = line angle - atan(cross / lookahead).
 * Defined everywhere, including at and past b -- unlike the bearing to b,
 * which is noisy close to b and flips 180 deg when b is passed. */
float nav_los_course_deg(nav_en_t a, nav_en_t b, nav_en_t p, float lookahead_m);
