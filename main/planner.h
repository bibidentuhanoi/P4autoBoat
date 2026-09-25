#pragma once

#include "path_follow.h"

/* The Planner slot (spec section 3): hands the path follower a short path in
 * local metres, from where the boat is to where the mission wants it.
 *
 * Today: the straight segment.  Next: a waypoint list.  Then the math
 * planner (SUSHI, JIRS 2026: wavefront + artificial potential fields on a
 * small grid around the boat, inflated by the hull, fed by ToF and the
 * camera) that bends the path around obstacles -- the same output type, so
 * the follower, the heading hold and the AUTO owner do not change. */
nav_path_t planner_straight(nav_en_t from, nav_en_t to);
