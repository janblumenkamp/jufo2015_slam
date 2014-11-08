/////////////////////////////////////////////////////////////////////////////////
/// Navigation - based on waypoints
/////////////////////////////////////////////////////////////////////////////////

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "slam.h"
#include "navigation_api.h"

extern nav_waypoint_t *nextWp; //Next waypoint in list (goal)

extern void navigate(slam_t *slam, mot_t *mot);

#endif // NAVIGATION_H
