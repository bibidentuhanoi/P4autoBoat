#include "planner.h"

nav_path_t planner_straight(nav_en_t from, nav_en_t to)
{
    nav_path_t p = {0};
    p.pts[0] = from;
    p.pts[1] = to;
    p.n = 2;
    return p;
}
