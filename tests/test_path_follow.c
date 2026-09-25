/* The path follower and the planner slot (2026-09-25): the ground a waypoint
 * mission and a SUSHI-style planner will stand on. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "path_follow.h"
#include "planner.h"

static float ang(float a, float b) { return fabsf(nav_wrap_180(a - b)); }

static void one_segment_is_exactly_los(void)
{
    const nav_en_t a = {3.0f, 9.0f}, b = {0.0f, 0.0f};
    const nav_path_t p = planner_straight(a, b);
    assert(p.n == 2 && p.pts[0].e == a.e && p.pts[1].n == b.n);
    path_follow_t f; path_follow_reset(&f);
    for (int i = -20; i <= 20; ++i) {
        const nav_en_t pos = {1.0f + 0.3f * (float)i, 5.0f - 0.2f * (float)i};
        const path_follow_out_t o = path_follow_step(&f, &p, pos, 3.0f);
        const nav_line_pos_t lp = nav_line_position(a, b, pos);
        assert(memcmp(&o.line, &lp, sizeof(lp)) == 0);            /* bit for bit */
        if (lp.progress <= 1.0f) assert(o.course_deg == nav_los_course_deg(a, b, pos, 3.0f));
        assert(o.last_segment && o.bearing_to_end_deg == nav_bearing_deg(pos, b));
    }
}

static void past_the_end_it_turns_back(void)
{
    const nav_path_t p = planner_straight((nav_en_t){0.0f, 10.0f}, (nav_en_t){0.0f, 0.0f});
    path_follow_t f; path_follow_reset(&f);
    const path_follow_out_t o = path_follow_step(&f, &p, (nav_en_t){2.0f, -3.0f}, 3.0f);
    assert(o.past_end);
    assert(ang(o.course_deg, nav_bearing_deg((nav_en_t){2.0f, -3.0f}, (nav_en_t){0.0f, 0.0f})) < 1e-3f);
}

static void segments_switch_at_their_finish_line_and_never_go_back(void)
{
    /* An L: 10 m north, then 10 m east. */
    nav_path_t p = {0};
    p.pts[0] = (nav_en_t){0.0f, 0.0f}; p.pts[1] = (nav_en_t){0.0f, 10.0f}; p.pts[2] = (nav_en_t){10.0f, 10.0f};
    p.n = 3;
    path_follow_t f; path_follow_reset(&f);
    path_follow_out_t o = path_follow_step(&f, &p, (nav_en_t){0.5f, 5.0f}, 3.0f);
    assert(f.seg == 0 && !o.last_segment && ang(o.course_deg, 0.0f) < 15.0f);   /* north-ish */
    o = path_follow_step(&f, &p, (nav_en_t){0.3f, 10.2f}, 3.0f);                 /* over the corner */
    assert(f.seg == 1 && o.last_segment && ang(o.course_deg, 90.0f) < 30.0f);   /* east-ish */
    o = path_follow_step(&f, &p, (nav_en_t){0.0f, 8.0f}, 3.0f);                  /* drifted back */
    assert(f.seg == 1);                                                          /* never re-entered */
}

static void a_boat_follows_a_zigzag_to_its_end(void)
{
    /* A unicycle at 0.4 m/s turning at most 15 deg/s -- the heading hold's
     * limit -- follows a 4-leg zigzag and ends at the last point. */
    nav_path_t p = {0};
    const nav_en_t pts[] = {{0, 0}, {0, 12}, {8, 18}, {8, 30}, {-4, 36}};
    for (int i = 0; i < 5; ++i) p.pts[i] = pts[i];
    p.n = 5;
    path_follow_t f; path_follow_reset(&f);
    nav_en_t pos = {0.0f, 0.0f};
    float hdg = 0.0f, max_cross = 0.0f, closest_end = 1e9f;
    for (int k = 0; k < 20 * 300; ++k) {
        const path_follow_out_t o = path_follow_step(&f, &p, pos, 3.0f);
        if (o.last_segment && o.past_end) break;
        const float err = nav_wrap_180(o.course_deg - hdg);
        hdg = nav_wrap_360(hdg + fmaxf(-0.75f, fminf(0.75f, err)));          /* 15 deg/s at 20 Hz */
        pos.e += 0.4f * 0.05f * sinf(hdg * 0.017453292f);
        pos.n += 0.4f * 0.05f * cosf(hdg * 0.017453292f);
        if (k > 40) max_cross = fmaxf(max_cross, fabsf(o.line.cross_m));
        closest_end = fminf(closest_end, nav_distance_m(pos, pts[4]));
    }
    assert(f.seg == 3);
    assert(closest_end < 0.5f);
    assert(max_cross < 3.0f);          /* the corners are cut, never lost */
}

static void a_bad_path_steers_nowhere(void)
{
    nav_path_t p = {0};
    path_follow_t f; path_follow_reset(&f);
    path_follow_out_t o = path_follow_step(&f, &p, (nav_en_t){1.0f, 1.0f}, 3.0f);
    assert(o.course_deg == 0.0f && !o.last_segment);
    p.n = NAV_PATH_MAX_POINTS + 1;
    o = path_follow_step(&f, &p, (nav_en_t){1.0f, 1.0f}, 3.0f);
    assert(o.course_deg == 0.0f);
    /* a zero-length segment in the middle is passed straight over */
    p.n = 3; p.pts[0] = (nav_en_t){0, 0}; p.pts[1] = (nav_en_t){0, 0}; p.pts[2] = (nav_en_t){0, 10};
    o = path_follow_step(&f, &p, (nav_en_t){0.0f, 1.0f}, 3.0f);
    assert(f.seg == 1 && ang(o.course_deg, 0.0f) < 1e-3f);
}

int main(void)
{
    one_segment_is_exactly_los();
    past_the_end_it_turns_back();
    segments_switch_at_their_finish_line_and_never_go_back();
    a_boat_follows_a_zigzag_to_its_end();
    a_bad_path_steers_nowhere();
    printf("path follower tests passed\n");
    return 0;
}
