#include "nav_geo.h"

#include <math.h>

#define NAV_PI 3.14159265358979323846
#define NAV_DEG2RAD (NAV_PI / 180.0)
#define NAV_DEG2RAD_F 0.017453292519943295f
#define NAV_RAD2DEG_F 57.29577951308232f

void nav_origin_init(nav_origin_t *o, double lat_deg, double lon_deg)
{
    if (!o) return;
    const double phi = lat_deg * NAV_DEG2RAD;
    o->lat0_deg = lat_deg;
    o->lon0_deg = lon_deg;
    /* WGS84 series for the length of one degree at this latitude. */
    o->m_per_deg_lat = 111132.954 - 559.822 * cos(2.0 * phi) + 1.175 * cos(4.0 * phi);
    o->m_per_deg_lon = 111412.84 * cos(phi) - 93.5 * cos(3.0 * phi) + 0.118 * cos(5.0 * phi);
}

static double wrap_dlon(double dlon)
{
    if (dlon > 180.0) dlon -= 360.0;
    if (dlon < -180.0) dlon += 360.0;
    return dlon;
}

nav_en_t nav_to_local(const nav_origin_t *o, double lat_deg, double lon_deg)
{
    nav_en_t p = {0.0f, 0.0f};
    if (!o) return p;
    p.e = (float)(wrap_dlon(lon_deg - o->lon0_deg) * o->m_per_deg_lon);
    p.n = (float)((lat_deg - o->lat0_deg) * o->m_per_deg_lat);
    return p;
}

void nav_to_latlon(const nav_origin_t *o, nav_en_t p, double *lat_deg, double *lon_deg)
{
    if (!o || !lat_deg || !lon_deg) return;
    *lat_deg = o->lat0_deg + (double)p.n / o->m_per_deg_lat;
    *lon_deg = o->lon0_deg + (double)p.e / o->m_per_deg_lon;
}

float nav_wrap_360(float deg)
{
    float w = fmodf(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    if (w >= 360.0f) w -= 360.0f;   /* fmodf of a tiny negative can round to 360 */
    return w;
}

float nav_wrap_180(float deg)
{
    float w = fmodf(deg + 180.0f, 360.0f);
    if (w < 0.0f) w += 360.0f;
    return w - 180.0f;
}

float nav_distance_m(nav_en_t a, nav_en_t b)
{
    return hypotf(b.e - a.e, b.n - a.n);
}

float nav_bearing_deg(nav_en_t from, nav_en_t to)
{
    const float de = to.e - from.e, dn = to.n - from.n;
    if (de == 0.0f && dn == 0.0f) return 0.0f;
    return nav_wrap_360(atan2f(de, dn) * NAV_RAD2DEG_F);
}

nav_en_t nav_offset(nav_en_t p, float bearing_deg, float dist_m)
{
    const float b = bearing_deg * NAV_DEG2RAD_F;
    nav_en_t q = { p.e + dist_m * sinf(b), p.n + dist_m * cosf(b) };
    return q;
}

nav_line_pos_t nav_line_position(nav_en_t a, nav_en_t b, nav_en_t p)
{
    nav_line_pos_t r = {0.0f, 0.0f, 1.0f, 0.0f};
    const float le = b.e - a.e, ln = b.n - a.n;
    const float len = hypotf(le, ln);
    r.length_m = len;
    const float pe = p.e - a.e, pn = p.n - a.n;
    if (len < 0.03f) {
        r.along_m = 0.0f;
        r.cross_m = 0.0f;
        r.progress = 1.0f;
        return r;
    }
    const float ue = le / len, un = ln / len;
    r.along_m = pe * ue + pn * un;
    /* Right-hand normal of a compass bearing a: (cos a, -sin a) = (un, -ue). */
    r.cross_m = pe * un - pn * ue;
    r.progress = r.along_m / len;
    return r;
}

float nav_los_course_deg(nav_en_t a, nav_en_t b, nav_en_t p, float lookahead_m)
{
    const float line = nav_bearing_deg(a, b);
    if (nav_distance_m(a, b) < 0.03f) return nav_bearing_deg(p, b);
    if (!(lookahead_m > 0.0f)) lookahead_m = 0.1f;
    const nav_line_pos_t lp = nav_line_position(a, b, p);
    const float correction = atanf(lp.cross_m / lookahead_m) * NAV_RAD2DEG_F;
    return nav_wrap_360(line - correction);
}
