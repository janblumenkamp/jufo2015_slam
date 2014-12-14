/////////////////////////////////////////////////////////////////////////////////
/// Navigation - based on waypoints
/////////////////////////////////////////////////////////////////////////////////

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "slam.h"
#include "navigation_api.h"

extern int16_t nextWP_ID; //Next waypoint in list (goal)

extern void navigate(slam_t *slam, mot_t *mot);

#endif // NAVIGATION_H
