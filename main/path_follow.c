#include "path_follow.h"

#include <stddef.h>

void path_follow_reset(path_follow_t *f)
{
    if (f) f->seg = 0;
}

path_follow_out_t path_follow_step(path_follow_t *f, const nav_path_t *path, nav_en_t pos,
                                   float lookahead_m)
{
    path_follow_out_t out = {0};
    if (!f || !path || path->n < 2 || path->n > NAV_PATH_MAX_POINTS) return out;
    const uint8_t last = (uint8_t)(path->n - 2);          /* index of the last segment */
    if (f->seg > last) f->seg = last;
    while (f->seg < last &&
           nav_line_position(path->pts[f->seg], path->pts[f->seg + 1], pos).progress >= 1.0f) {
        f->seg++;
    }
    const nav_en_t a = path->pts[f->seg];
    const nav_en_t b = path->pts[f->seg + 1];
    out.line = nav_line_position(a, b, pos);
    out.last_segment = f->seg == last;
    out.bearing_to_end_deg = nav_bearing_deg(pos, path->pts[path->n - 1]);
    out.past_end = out.last_segment && out.line.progress > 1.0f;
    out.course_deg = out.past_end ? out.bearing_to_end_deg
                                  : nav_los_course_deg(a, b, pos, lookahead_m);
    return out;
}
