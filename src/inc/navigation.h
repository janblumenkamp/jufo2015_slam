/////////////////////////////////////////////////////////////////////////////////
/// Navigation - based on waypoints
/////////////////////////////////////////////////////////////////////////////////

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "slam.h"
#include "navigation_api.h"

enum{
	NAV_MODE_STOP,
	NAV_MODE_EXPLORATION,
	NAV_MODE_WAYPOINT,
	NAV_MODE_MANUAL
};

extern uint8_t nav_mode;

extern int16_t nextWP_ID; //Next waypoint in list (goal)

extern void navigate(slam_t *slam, mot_t *mot);

#endif // NAVIGATION_H
