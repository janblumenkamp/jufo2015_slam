
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "navigation.h"
#include "navigation_api.h"
#include "outf.h"
#include "slam.h"
#include "math.h"
#include "stdlib.h"
#include "gui.h"

int16_t nextWP_ID = -1; //Next waypoint in list (goal)
nav_waypoint_t *nextWp; //Store also a pointer to the next waypoint (not only id, this pointer i important for the robot program
float nextWp_dist = 0; //Dist to next waypoint

void navigate(slam_t *slam, mot_t *mot)
{
	float wp_dx, wp_dy;
	float psi = 0;

	if(nav_wpStart != NULL)
	{
		if(nextWP_ID == -1) //List created. Now we can start to navigate! This only happens once
		{
			nextWP_ID = nav_wpStart->id;
			nextWp = nav_wpStart;
		}
		else if(nextWP_ID != -1)
		{
			wp_dx = slam->robot_pos.coord.x - nextWp->x; //Convert root of cartesian coordinate system to the robot position (robot is now the root and wp_dx/dy are the coordinates of the waypoint)
			wp_dy = slam->robot_pos.coord.y - nextWp->y;

			nextWp_dist = sqrtf((wp_dx * wp_dx) + (wp_dy * wp_dy)); //Calculate dist to waypoint

			if(nextWp_dist < 200) //20cm close to the waypoint
			{
				if(nextWp->next != NULL && nextWp->next->id != -1) //We are not at the last waypoint in the list
				{
					nextWP_ID = nextWp->next->id; //Switch to next waypoint
					nextWp = nav_getWaypoint(nextWP_ID);
				}
				else
				{
					mot->speed_l_to = 0; //Stop robot
					mot->speed_r_to = 0;
				}
			}
			else //Calculate motor data
			{
				if(wp_dy >= 0)	psi = acosf(wp_dx / nextWp_dist);
				else			psi = acosf(-(wp_dx / nextWp_dist)) - M_PI;

				psi *= -(180/M_PI); //Convert to degree
				psi = slam->robot_pos.psi - psi;
				if(psi > 180)
					//psi = 360 - psi;
					psi -= 360;

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

				//printf("Abweichung wp: %i, speedL: %i, speedR: %i\n", (int)psi, speedvar_l, speedvar_r);
			}
		}
	}
	else //Drive random, avoid obstacles (exploration mode)
	{
		uint16_t lidar_min = 0xffff;
		int16_t lidar_min_i = 0; //Index of lowest lidar entry

		for(int16_t i = 90; i < 270; i++)
		{
			if(slam->sensordata.lidar[i] > LASERSCAN_NODATA && slam->sensordata.lidar[i] < lidar_min)
			{
				lidar_min_i = i - 180; //Lidar index: now 0 means front, -90 right and 90 left
				lidar_min = slam->sensordata.lidar[i];
			}
		}

		if(lidar_min < 300)
		{
			if(lidar_min_i > 0)
			{
				mot->speed_l_to = 10;
				mot->speed_r_to = -10;
			}
			else
			{
				mot->speed_l_to = -10;
				mot->speed_r_to = 10;
			}
		}
		else
		{
			mot->speed_l_to = 20;
			mot->speed_r_to = 20;
		}
	}
}
