/////////////////////////////////////////////////////////////////////////////////
/// Navigation API - all the waypoint stack management and waypoint management functions
/////////////////////////////////////////////////////////////////////////////////

#ifndef NAVIGATION_API_H
#define NAVIGATION_API_H

#define WP_STACKSIZE 100

typedef struct nav_waypoint {
	int16_t x, y; //Position in field
	int8_t z; //which stage
	int16_t id; //ID of waypoint. Nessesary e.g. to delete a selected waypoint in the middle of the list. Always increasing.
	struct nav_waypoint *previous; //Points to the previous element in the list
	struct nav_waypoint *next; //Points to the next element in the list
} nav_waypoint_t;

extern nav_waypoint_t *nav_wpStart, *nav_wpEnd; //Pointer to first and last waypoint
extern u_int16_t nav_wpAmount; //Contains the amount of the current used waypoints

extern void nav_resetWp(nav_waypoint_t *wp);

extern void nav_initWaypointStack(void);

extern nav_waypoint_t *nav_stackFindEmptyElement(void);

extern void nav_stackFree(nav_waypoint_t *wp);

extern void nav_attachWaypoint(nav_waypoint_t *wp);

extern void nav_deleteWaypoint(int16_t id);

extern nav_waypoint_t *nav_getWaypoint(int16_t id);

#endif // NAVIGATION_API_H
