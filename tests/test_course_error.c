#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "course_error.h"

static int close_to(double a, double b, double eps) { return fabs(a - b) <= eps; }

static void gates_reject_slow_turning_and_unknown_quality(void)
{
    const course_err_cfg_t c = course_err_cfg_default();
    assert(course_err_sample_ok(&c, 0.38f, 0.05f, 1.0f));      /* asin(0.13) = 7.6 deg */
    assert(!course_err_sample_ok(&c, 0.20f, 0.05f, 1.0f));     /* too slow */
    assert(!course_err_sample_ok(&c, 0.38f, 0.05f, 8.0f));     /* turning */
    assert(!course_err_sample_ok(&c, 0.38f, 0.0f, 1.0f));      /* sAcc unknown */
    assert(course_err_sample_ok(&c, 0.38f, 0.12f, 1.0f));      /* asin(0.32) = 18 deg: kept */
    assert(course_err_sample_ok(&c, 0.38f, 0.26f, 1.0f));      /* asin(0.68) = 43 deg: kept */
    assert(!course_err_sample_ok(&c, 0.38f, 0.28f, 1.0f));     /* asin(0.74) = 47 deg */
    assert(!course_err_sample_ok(&c, 0.38f, 0.40f, 1.0f));     /* sAcc > speed */
    assert(!course_err_sample_ok(&c, NAN, 0.05f, 1.0f));
}

static void samples_wrap_the_short_way(void)
{
    assert(close_to(course_err_sample(5.0f, 355.0f), 10.0, 1e-4));
    assert(close_to(course_err_sample(355.0f, 5.0f), -10.0, 1e-4));
    assert(close_to(course_err_sample(213.0f, 168.0f), 45.0, 1e-4));   /* 09-21: old compass */
}

static course_err_mean_t filled(float value, float jitter, int n, float step_m)
{
    course_err_mean_t m;
    course_err_mean_reset(&m);
    for (int i = 0; i < n; ++i) {
        const float s = value + ((i % 2) ? jitter : -jitter);
        course_err_mean_add(&m, s, 0.0f, step_m * (float)i);
    }
    return m;
}

static void the_outbound_mean_needs_count_travel_and_consistency(void)
{
    const course_err_cfg_t c = course_err_cfg_default();
    course_err_mean_t m = filled(20.0f, 5.0f, 60, 0.1f);          /* 5.9 m of travel */
    course_err_result_t r = course_err_mean_result(&m, &c);
    assert(r.valid && close_to(r.mean_deg, 20.0, 0.05) && r.std_deg < 6.0f && !r.compass_bad);
    m = filled(20.0f, 5.0f, 40, 0.2f);                             /* too few */
    assert(!course_err_mean_result(&m, &c).valid);
    m = filled(20.0f, 5.0f, 60, 0.05f);                            /* 3 m only */
    assert(!course_err_mean_result(&m, &c).valid);
    m = filled(20.0f, 40.0f, 60, 0.1f);                            /* inconsistent */
    assert(!course_err_mean_result(&m, &c).valid);
    /* The mean wraps: +175 and -175 average to 180, not 0. */
    m = filled(180.0f, 5.0f, 60, 0.1f);
    r = course_err_mean_result(&m, &c);
    assert(r.valid && fabsf(fabsf(r.mean_deg) - 60.0f) < 1e-4f);   /* 180 clamped to +-60 */
    assert(r.compass_bad);
    /* ArduPilot's 45 deg: bad above it, not at 40. */
    m = filled(50.0f, 2.0f, 60, 0.1f);
    assert(course_err_mean_result(&m, &c).compass_bad);
    m = filled(40.0f, 2.0f, 60, 0.1f);
    assert(!course_err_mean_result(&m, &c).compass_bad);
}

static void the_return_filter_follows_slowly_the_short_way(void)
{
    const course_err_cfg_t c = course_err_cfg_default();          /* tau 8 s */
    float b = 20.0f;
    for (int i = 0; i < 80; ++i) b = course_err_follow(b, -20.0f, 0.1f, &c);   /* 8 s */
    assert(b < 20.0f - 40.0f * 0.55f && b > -20.0f);              /* ~63 % of the way */
    b = 170.0f;                                                    /* beyond the clamp */
    b = course_err_follow(b, 170.0f, 0.1f, &c);
    assert(close_to(b, 60.0, 1e-4));
    assert(close_to(course_err_follow(10.0f, NAN, 0.1f, &c), 10.0, 1e-6));
    assert(close_to(course_err_follow(10.0f, 30.0f, 0.0f, &c), 10.0, 1e-6));
}

int main(void)
{
    gates_reject_slow_turning_and_unknown_quality();
    samples_wrap_the_short_way();
    the_outbound_mean_needs_count_travel_and_consistency();
    the_return_filter_follows_slowly_the_short_way();
    printf("course_error tests passed\n");
    return 0;
}
