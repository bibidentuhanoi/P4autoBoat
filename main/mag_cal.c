#include "mag_cal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PI_D 3.14159265358979323846

static bool finite_all(const float *v, int n)
{
    for (int i = 0; i < n; i++) {
        if (!isfinite(v[i])) return false;
    }
    return true;
}

void mag_cal_identity(mag_cal_2d_t *cal)
{
    if (!cal) return;
    memset(cal, 0, sizeof(*cal));
    cal->soft[0] = 1.0f;
    cal->soft[3] = 1.0f;
    /* radius 0 = "never calibrated": nothing to compare field strength to */
}

bool mag_cal_valid(const mag_cal_2d_t *cal)
{
    if (!cal) return false;
    if (!finite_all(cal->center, 2) || !finite_all(cal->soft, 4) ||
        !isfinite(cal->radius)) {
        return false;
    }
    if (fabsf(cal->center[0]) > 1e5f || fabsf(cal->center[1]) > 1e5f) return false;
    for (int i = 0; i < 4; i++) {
        if (fabsf(cal->soft[i]) > 10.0f) return false;
    }
    /* A mirror image (det < 0) would make the heading run backwards. */
    float det = cal->soft[0] * cal->soft[3] - cal->soft[1] * cal->soft[2];
    if (!(det > 0.05f && det < 20.0f)) return false;
    return cal->radius >= 0.0f && cal->radius < MAG_CAL_RADIUS_MAX_LSB * 2.0f;
}

void mag_cal_apply(const mag_cal_2d_t *cal, float mx, float my,
                   float *out_x, float *out_y)
{
    float dx = mx - cal->center[0];
    float dy = my - cal->center[1];
    *out_x = cal->soft[0] * dx + cal->soft[1] * dy;
    *out_y = cal->soft[2] * dx + cal->soft[3] * dy;
}

float mag_cal_heading_deg(float x, float y)
{
    float h = atan2f(y, x) * (float)(180.0 / PI_D);
    if (h < 0.0f) h += 360.0f;
    if (h >= 360.0f) h -= 360.0f;
    return h;
}

float mag_cal_tilt_deg(const float accel[3], const float ref[3])
{
    double dot = 0.0, na = 0.0, nr = 0.0;
    for (int i = 0; i < 3; i++) {
        dot += (double)accel[i] * ref[i];
        na += (double)accel[i] * accel[i];
        nr += (double)ref[i] * ref[i];
    }
    if (!(na > 0.0) || !(nr > 0.0) || !isfinite(dot)) return 180.0f;
    double c = dot / sqrt(na * nr);
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return (float)(acos(c) * 180.0 / PI_D);
}

/* ---- Step 1: hold still ------------------------------------------------ */

void mag_still_init(mag_still_t *s, int window, int collect,
                    float accel_std_max, float gyro_std_max)
{
    memset(s, 0, sizeof(*s));
    s->window = window > 1 ? window : 2;
    s->collect = collect > s->window ? collect : s->window;
    s->accel_std_max = accel_std_max;
    s->gyro_std_max = gyro_std_max;
    s->state = MAG_STILL_WAITING;
}

static double window_std(double sum, double sum_sq, int n)
{
    double mean = sum / n;
    double var = sum_sq / n - mean * mean;
    return var > 0.0 ? sqrt(var) : 0.0;
}

mag_still_state_t mag_still_add(mag_still_t *s, const int16_t accel[3],
                                const int16_t gyro[3])
{
    if (s->state == MAG_STILL_DONE) return s->state;

    for (int i = 0; i < 3; i++) {
        double a = accel[i], g = gyro[i];
        s->win_a[i] += a;
        s->win_aa[i] += a * a;
        s->win_g[i] += g;
        s->win_gg[i] += g * g;
    }
    if (++s->n_win < s->window) return s->state;

    bool still = true;
    for (int i = 0; i < 3; i++) {
        if (window_std(s->win_a[i], s->win_aa[i], s->n_win) > s->accel_std_max ||
            window_std(s->win_g[i], s->win_gg[i], s->n_win) > s->gyro_std_max) {
            still = false;
        }
    }

    if (still) {
        /* Only whole still windows count, so a bump part-way through a
         * window can never leak into the average. */
        for (int i = 0; i < 3; i++) {
            s->col_a[i] += s->win_a[i];
            s->col_g[i] += s->win_g[i];
        }
        s->n_col += s->n_win;
        s->state = (s->n_col >= s->collect) ? MAG_STILL_DONE : MAG_STILL_COLLECTING;
    } else {
        memset(s->col_a, 0, sizeof(s->col_a));
        memset(s->col_g, 0, sizeof(s->col_g));
        s->n_col = 0;
        s->state = MAG_STILL_WAITING;
    }

    s->n_win = 0;
    memset(s->win_a, 0, sizeof(s->win_a));
    memset(s->win_aa, 0, sizeof(s->win_aa));
    memset(s->win_g, 0, sizeof(s->win_g));
    memset(s->win_gg, 0, sizeof(s->win_gg));
    return s->state;
}

float mag_still_progress(const mag_still_t *s)
{
    if (s->state == MAG_STILL_DONE) return 1.0f;
    float p = (float)s->n_col / (float)s->collect;
    return p > 1.0f ? 1.0f : p;
}

void mag_still_result(const mag_still_t *s, float gyro_mean[3], float accel_mean[3])
{
    for (int i = 0; i < 3; i++) {
        gyro_mean[i] = s->n_col ? (float)(s->col_g[i] / s->n_col) : 0.0f;
        accel_mean[i] = s->n_col ? (float)(s->col_a[i] / s->n_col) : 0.0f;
    }
}

/* ---- Step 2: spin flat ------------------------------------------------- */

void mag_circle_init(mag_circle_t *c, float *x, float *y, float *turn, int cap)
{
    memset(c, 0, sizeof(*c));
    c->x = x;
    c->y = y;
    c->turn = turn;
    c->cap = cap;
    c->keep_every = 1;
    c->min[0] = c->min[1] = INFINITY;
    c->max[0] = c->max[1] = -INFINITY;
}

void mag_circle_add(mag_circle_t *c, float mx, float my,
                    float yaw_rate_dps, float dt_s)
{
    /* The gyro keeps counting even when a compass reading is missing, or
     * the stored turn angles would fall behind the real rotation. */
    if (isfinite(yaw_rate_dps) && isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.5f) {
        c->turn_deg += yaw_rate_dps * dt_s;
    }
    if (!isfinite(mx) || !isfinite(my)) return;
    c->total++;
    if (mx < c->min[0]) c->min[0] = mx;
    if (mx > c->max[0]) c->max[0] = mx;
    if (my < c->min[1]) c->min[1] = my;
    if (my > c->max[1]) c->max[1] = my;

    if (++c->skip < c->keep_every) return;
    c->skip = 0;

    if (c->n >= c->cap) {
        /* Full: keep every other sample (time order preserved) and store
         * half as often from now on, so a slow spin still fits. */
        int half = c->cap / 2;
        for (int i = 0; i < half; i++) {
            c->x[i] = c->x[2 * i];
            c->y[i] = c->y[2 * i];
            c->turn[i] = c->turn[2 * i];
        }
        c->n = half;
        c->keep_every *= 2;
    }
    c->x[c->n] = mx;
    c->y[c->n] = my;
    c->turn[c->n] = c->turn_deg;
    c->n++;
}

static int bin_of(float dx, float dy)
{
    double a = atan2((double)dy, (double)dx);           /* -pi..pi */
    int b = (int)((a + PI_D) / (2.0 * PI_D) * MAG_CAL_BINS);
    if (b < 0) b = 0;
    if (b >= MAG_CAL_BINS) b = MAG_CAL_BINS - 1;
    return b;
}

uint32_t mag_circle_live_mask(const mag_circle_t *c)
{
    if (c->n == 0) return 0;
    float cx = 0.5f * (c->min[0] + c->max[0]);
    float cy = 0.5f * (c->min[1] + c->max[1]);
    uint32_t mask = 0;
    for (int i = 0; i < c->n; i++) {
        mask |= 1u << bin_of(c->x[i] - cx, c->y[i] - cy);
    }
    return mask;
}

static int popcount32(uint32_t v)
{
    int n = 0;
    while (v) { v &= v - 1; n++; }
    return n;
}

bool mag_circle_complete(const mag_circle_t *c)
{
    return c->n >= MAG_CAL_MIN_SAMPLES &&
           fabsf(c->turn_deg) >= MAG_CAL_MIN_TURN_DEG &&
           popcount32(mag_circle_live_mask(c)) == MAG_CAL_BINS;
}

/* ---- Fit and judge ----------------------------------------------------- */

/* Gaussian elimination with partial pivoting, 5x5.  False if singular. */
static bool solve5(double a[5][5], double b[5], double x[5])
{
    double scale = 0.0;
    for (int i = 0; i < 5; i++) {
        if (fabs(a[i][i]) > scale) scale = fabs(a[i][i]);
    }
    if (!(scale > 0.0)) return false;
    for (int col = 0; col < 5; col++) {
        int piv = col;
        for (int r = col + 1; r < 5; r++) {
            if (fabs(a[r][col]) > fabs(a[piv][col])) piv = r;
        }
        if (fabs(a[piv][col]) < 1e-10 * scale) return false;
        if (piv != col) {
            for (int k = 0; k < 5; k++) {
                double t = a[col][k]; a[col][k] = a[piv][k]; a[piv][k] = t;
            }
            double t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        for (int r = 0; r < 5; r++) {
            if (r == col) continue;
            double f = a[r][col] / a[col][col];
            for (int k = col; k < 5; k++) a[r][k] -= f * a[col][k];
            b[r] -= f * b[col];
        }
    }
    for (int i = 0; i < 5; i++) x[i] = b[i] / a[i][i];
    return true;
}

static float wrap180f(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* Fit A u^2 + B uv + C v^2 + D u + E v = 1 on centred, scaled data, then
 * turn the ellipse into centre + symmetric square-root correction. */
static bool fit_ellipse(const float *x, const float *y, int n, mag_fit_t *out)
{
    double mx = 0.0, my = 0.0;
    for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; }
    mx /= n;
    my /= n;
    double s = 0.0;
    for (int i = 0; i < n; i++) {
        double dx = x[i] - mx, dy = y[i] - my;
        s += dx * dx + dy * dy;
    }
    s = sqrt(s / n);
    if (!(s > 1e-3) || !isfinite(s)) return false;

    double ata[5][5] = {{0}};
    double atb[5] = {0};
    for (int i = 0; i < n; i++) {
        double u = (x[i] - mx) / s, v = (y[i] - my) / s;
        double row[5] = {u * u, u * v, v * v, u, v};
        for (int r = 0; r < 5; r++) {
            atb[r] += row[r];
            for (int k = 0; k < 5; k++) ata[r][k] += row[r] * row[k];
        }
    }
    double p[5];
    if (!solve5(ata, atb, p)) return false;
    double A = p[0], B = p[1], C = p[2], D = p[3], E = p[4];

    double det = A * C - 0.25 * B * B;
    if (!(A > 0.0) || !(det > 0.0)) return false;           /* not an ellipse */
    double pu = -0.5 * (C * D - 0.5 * B * E) / det;
    double pv = -0.5 * (A * E - 0.5 * B * D) / det;
    double k = 1.0 + (A * pu * pu + B * pu * pv + C * pv * pv);
    if (!(k > 0.0)) return false;

    /* Ellipse in raw counts: (p - c)^T Q (p - c) = 1 */
    double q00 = A / (k * s * s);
    double q01 = 0.5 * B / (k * s * s);
    double q11 = C / (k * s * s);
    double trq = q00 + q11;
    double detq = q00 * q11 - q01 * q01;
    if (!(detq > 0.0)) return false;
    double disc = sqrt(fmax(0.25 * trq * trq - detq, 0.0));
    double lmax = 0.5 * trq + disc, lmin = 0.5 * trq - disc;
    if (!(lmin > 0.0)) return false;

    double radius = pow(detq, -0.25);                        /* sqrt(a1 * a2) */
    double sq = sqrt(detq);
    double norm = sqrt(trq + 2.0 * sq);                      /* sqrtm of 2x2 SPD */
    out->cal.center[0] = (float)(mx + s * pu);
    out->cal.center[1] = (float)(my + s * pv);
    out->cal.soft[0] = (float)(radius * (q00 + sq) / norm);
    out->cal.soft[1] = (float)(radius * q01 / norm);
    out->cal.soft[2] = (float)(radius * q01 / norm);
    out->cal.soft[3] = (float)(radius * (q11 + sq) / norm);
    out->cal.radius = (float)radius;
    out->axis_ratio = (float)sqrt(lmax / lmin);
    return mag_cal_valid(&out->cal);
}

/* How far the corrected compass heading strays from the gyro over the spin.
 * Heading grows clockwise, gyro yaw grows to the left, hence the minus. */
static void gyro_check(const mag_cal_2d_t *cal, const float *x, const float *y,
                       const float *turn, int n, float *scale, float *dev_max)
{
    double sg = 0, sm = 0, sgg = 0, sgm = 0;
    float prev_h = 0.0f, mag = 0.0f;
    for (int i = 0; i < n; i++) {
        float cx, cy;
        mag_cal_apply(cal, x[i], y[i], &cx, &cy);
        float h = mag_cal_heading_deg(cx, cy);
        if (i > 0) mag += -wrap180f(h - prev_h);
        prev_h = h;
        double g = turn[i] - turn[0];
        sg += g; sm += mag; sgg += g * g; sgm += g * mag;
    }
    double den = n * sgg - sg * sg;
    if (!(den > 0.0)) { *scale = 0.0f; *dev_max = INFINITY; return; }
    double kk = (n * sgm - sg * sm) / den;
    double bb = (sm - kk * sg) / n;
    *scale = (float)kk;

    mag = 0.0f;
    float worst = 0.0f;
    for (int i = 0; i < n; i++) {
        float cx, cy;
        mag_cal_apply(cal, x[i], y[i], &cx, &cy);
        float h = mag_cal_heading_deg(cx, cy);
        if (i > 0) mag += -wrap180f(h - prev_h);
        prev_h = h;
        float d = fabsf(mag - (float)(kk * (turn[i] - turn[0]) + bb));
        if (d > worst) worst = d;
    }
    *dev_max = worst;
}

float mag_cal_tolerance_deg(float requested)
{
    if (!isfinite(requested) || requested <= 0.0f) return MAG_CAL_GYRO_DEV_RELAXED;
    if (requested < MAG_CAL_GYRO_DEV_MIN) return MAG_CAL_GYRO_DEV_MIN;
    if (requested > MAG_CAL_GYRO_DEV_MAX) return MAG_CAL_GYRO_DEV_MAX;
    return requested;
}

mag_cal_verdict_t mag_cal_fit_and_judge(const float *x, const float *y,
                                        const float *turn, int n,
                                        float max_gyro_dev_deg, mag_fit_t *out)
{
    const float max_dev = mag_cal_tolerance_deg(max_gyro_dev_deg);
    memset(out, 0, sizeof(*out));
    mag_cal_identity(&out->cal);
    if (!x || !y || !turn || n < MAG_CAL_MIN_SAMPLES) return MAG_CAL_FAIL_TOO_FEW;
    if (!fit_ellipse(x, y, n, out)) {
        mag_cal_identity(&out->cal);
        return MAG_CAL_FAIL_FIT;
    }

    double ss = 0.0;
    uint32_t mask = 0;
    for (int i = 0; i < n; i++) {
        float cx, cy;
        mag_cal_apply(&out->cal, x[i], y[i], &cx, &cy);
        double rel = sqrt((double)cx * cx + (double)cy * cy) / out->cal.radius - 1.0;
        ss += rel * rel;
        mask |= 1u << bin_of(cx, cy);
    }
    out->rms = (float)sqrt(ss / n);
    out->mask = mask;
    out->bins = popcount32(mask);
    gyro_check(&out->cal, x, y, turn, n, &out->gyro_scale, &out->gyro_dev_deg);

    if (out->cal.radius < MAG_CAL_RADIUS_MIN_LSB ||
        out->cal.radius > MAG_CAL_RADIUS_MAX_LSB) return MAG_CAL_FAIL_FIELD;
    if (out->axis_ratio > MAG_CAL_MAX_AXIS_RATIO) return MAG_CAL_FAIL_DISTORTED;
    if (out->rms > MAG_CAL_MAX_RMS) return MAG_CAL_FAIL_NOISY;
    if (out->bins < MAG_CAL_MIN_BINS) return MAG_CAL_FAIL_COVERAGE;
    if (!(out->gyro_scale >= MAG_CAL_GYRO_SCALE_MIN &&
          out->gyro_scale <= MAG_CAL_GYRO_SCALE_MAX) ||
        !(out->gyro_dev_deg <= max_dev)) return MAG_CAL_FAIL_GYRO;
    return MAG_CAL_PASS;
}

const char *mag_cal_verdict_text(mag_cal_verdict_t v)
{
    switch (v) {
    case MAG_CAL_PASS:           return "PASS";
    case MAG_CAL_FAIL_TOO_FEW:   return "not enough compass readings";
    case MAG_CAL_FAIL_FIT:       return "readings do not form a circle";
    case MAG_CAL_FAIL_DISTORTED: return "field too squashed - metal too close";
    case MAG_CAL_FAIL_NOISY:     return "readings too scattered";
    case MAG_CAL_FAIL_FIELD:     return "field strength not Earth-like";
    case MAG_CAL_FAIL_COVERAGE:  return "part of the circle missing";
    case MAG_CAL_FAIL_GYRO:      return "compass turn disagrees with gyro";
    }
    return "unknown";
}
