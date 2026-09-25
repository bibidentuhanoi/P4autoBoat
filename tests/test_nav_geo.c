#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "nav_geo.h"

static int close_to(double a, double b, double eps) { return fabs(a - b) <= eps; }

/* The lake (Hanoi). */
#define LAT0 21.0417483
#define LON0 105.8862132

static void round_trip_is_exact_to_a_millimetre(void)
{
    nav_origin_t o;
    nav_origin_init(&o, LAT0, LON0);
    const nav_en_t p = {12.34f, -56.78f};
    double lat, lon;
    nav_to_latlon(&o, p, &lat, &lon);
    const nav_en_t q = nav_to_local(&o, lat, lon);
    assert(close_to(q.e, p.e, 0.001) && close_to(q.n, p.n, 0.001));
    const nav_en_t z = nav_to_local(&o, LAT0, LON0);
    assert(z.e == 0.0f && z.n == 0.0f);
}

static void metres_per_degree_match_the_haversine(void)
{
    /* 50 m north and 50 m east, against the great-circle distance. */
    nav_origin_t o;
    nav_origin_init(&o, LAT0, LON0);
    double lat, lon;
    nav_to_latlon(&o, (nav_en_t){50.0f, 50.0f}, &lat, &lon);
    const double R = 6371008.8, d2r = 3.14159265358979323846 / 180.0;
    const double dlat = (lat - LAT0) * d2r, dlon = (lon - LON0) * d2r;
    const double a = sin(dlat / 2) * sin(dlat / 2) +
                     cos(LAT0 * d2r) * cos(lat * d2r) * sin(dlon / 2) * sin(dlon / 2);
    const double hav = 2 * R * atan2(sqrt(a), sqrt(1 - a));
    assert(close_to(hav, hypot(50.0, 50.0), 0.35));   /* sphere vs ellipsoid < 0.5 % */
}

static void wraps_cover_both_edges(void)
{
    assert(nav_wrap_360(360.0f) == 0.0f);
    assert(nav_wrap_360(-1e-6f) >= 0.0f && nav_wrap_360(-1e-6f) < 360.0f);
    assert(close_to(nav_wrap_360(-90.0f), 270.0, 1e-4));
    assert(close_to(nav_wrap_360(725.0f), 5.0, 1e-3));
    assert(close_to(nav_wrap_180(190.0f), -170.0, 1e-4));
    assert(close_to(nav_wrap_180(-190.0f), 170.0, 1e-4));
    assert(close_to(nav_wrap_180(180.0f), -180.0, 1e-4));
}

static void bearings_and_offsets_use_compass_degrees(void)
{
    const nav_en_t o = {0.0f, 0.0f};
    assert(close_to(nav_bearing_deg(o, (nav_en_t){0.0f, 10.0f}), 0.0, 1e-4));
    assert(close_to(nav_bearing_deg(o, (nav_en_t){10.0f, 0.0f}), 90.0, 1e-4));
    assert(close_to(nav_bearing_deg(o, (nav_en_t){0.0f, -10.0f}), 180.0, 1e-4));
    assert(close_to(nav_bearing_deg(o, (nav_en_t){-10.0f, 0.0f}), 270.0, 1e-4));
    assert(nav_bearing_deg(o, o) == 0.0f);
    const nav_en_t p = nav_offset(o, 90.0f, 10.0f);
    assert(close_to(p.e, 10.0, 1e-4) && close_to(p.n, 0.0, 1e-4));
    const nav_en_t q = nav_offset(o, 225.0f, 2.0f);
    assert(close_to(q.e, -1.41421, 1e-4) && close_to(q.n, -1.41421, 1e-4));
    assert(close_to(nav_distance_m(o, (nav_en_t){3.0f, 4.0f}), 5.0, 1e-5));
}

static void line_position_gives_along_cross_and_the_finish_line(void)
{
    const nav_en_t a = {0.0f, 0.0f}, b = {0.0f, 10.0f};        /* due north */
    nav_line_pos_t r = nav_line_position(a, b, (nav_en_t){1.0f, 5.0f});
    assert(close_to(r.along_m, 5.0, 1e-5) && close_to(r.cross_m, 1.0, 1e-5));   /* east = right */
    assert(close_to(r.progress, 0.5, 1e-6) && close_to(r.length_m, 10.0, 1e-5));
    r = nav_line_position(a, b, (nav_en_t){-2.0f, 12.0f});
    assert(close_to(r.cross_m, -2.0, 1e-5) && r.progress > 1.0f);                /* past the line */
    /* A south-going line: east is now on the LEFT. */
    r = nav_line_position(b, a, (nav_en_t){1.0f, 5.0f});
    assert(close_to(r.cross_m, -1.0, 1e-5) && close_to(r.progress, 0.5, 1e-6));
    /* Degenerate line: already there. */
    r = nav_line_position(a, (nav_en_t){0.01f, 0.01f}, (nav_en_t){5.0f, 5.0f});
    assert(r.progress == 1.0f);
}

static void los_steers_back_to_the_line_and_never_flips(void)
{
    const nav_en_t a = {0.0f, 0.0f}, b = {0.0f, 10.0f};
    assert(close_to(nav_los_course_deg(a, b, (nav_en_t){0.0f, 5.0f}, 3.0f), 0.0, 1e-3));
    /* 3 m right of a north line with 3 m lookahead: aim 45 deg left. */
    assert(close_to(nav_los_course_deg(a, b, (nav_en_t){3.0f, 5.0f}, 3.0f), 315.0, 1e-3));
    assert(close_to(nav_los_course_deg(a, b, (nav_en_t){-3.0f, 5.0f}, 3.0f), 45.0, 1e-3));
    /* At and past the end of the line: still the line's course, no 180 flip. */
    assert(close_to(nav_los_course_deg(a, b, (nav_en_t){0.0f, 10.0f}, 3.0f), 0.0, 1e-3));
    assert(close_to(nav_los_course_deg(a, b, (nav_en_t){0.0f, 13.0f}, 3.0f), 0.0, 1e-3));
    /* South-going line, boat east of it (its left): steer right, toward the west. */
    const float c = nav_los_course_deg(b, a, (nav_en_t){1.0f, 5.0f}, 3.0f);
    assert(c > 180.0f && c < 200.0f);
}

int main(void)
{
    round_trip_is_exact_to_a_millimetre();
    metres_per_degree_match_the_haversine();
    wraps_cover_both_edges();
    bearings_and_offsets_use_compass_degrees();
    line_position_gives_along_cross_and_the_finish_line();
    los_steers_back_to_the_line_and_never_flips();
    printf("nav_geo tests passed\n");
    return 0;
}
