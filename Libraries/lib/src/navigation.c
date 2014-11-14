
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "navigation.h"
#include "navigation_api.h"
#include "printf.h"
#include "slam.h"
#include "math.h"
#include "stdlib.h"
#include "gui.h"

nav_waypoint_t *nextWp; //Next waypoint in list (goal)
float nextWp_dist = 0; //Dist to next waypoint

void navigate(slam_t *slam, mot_t *mot)
{
	float wp_dx, wp_dy;
	float psi = 0;

	if(nav_wpStart != NULL && nextWp == NULL) //List created. Now we can start to navigate!
	{
		nextWp = nav_wpStart;
	}
	else if(nextWp != NULL)
	{
		wp_dx = slam->robot_pos.coord.x - nextWp->x; //Convert root of cartesian coordinate system to the robot position (robot is now the root and wp_dx/dy are the coordinates of the waypoint)
		wp_dy = slam->robot_pos.coord.y - nextWp->y;

		nextWp_dist = sqrtf((wp_dx * wp_dx) + (wp_dy * wp_dy)); //Calculate dist to waypoint

		if(nextWp_dist < 200) //20cm close to the waypoint
		{
			nextWp = nextWp->next; //Switch to next waypoint
		}
		else //Calculate motor data
		{
			if(wp_dy >= 0)	psi = acosf(wp_dx / nextWp_dist);
			else			psi = acosf(-(wp_dx / nextWp_dist)) - M_PI;

			psi *= -(180/M_PI); //Convert to degree
			psi = slam->robot_pos.psi - psi;
			if(psi > 180)
				psi = 360 - psi;

			int16_t speedvar_l = 20 - (psi/3);
			int16_t speedvar_r = 20 + (psi/3);

			if(speedvar_l < -20)
				speedvar_l = -20;
			else if(speedvar_l > 20)
				speedvar_l = 20;
			if(speedvar_r < -20)
				speedvar_r = -20;
			else if(speedvar_r > 20)
				speedvar_r = 20;

			mot->speed_l_to = speedvar_l;
			mot->speed_r_to = speedvar_r;

			//printf("Abweichung wp: %i, speedL: %i, speedR: %i\n\r", (int)psi, speedvar_l, speedvar_r);
		}
	}
}
