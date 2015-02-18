#include "slamdefs.h"
#include <math.h>
#include <stdlib.h>
#include "outf.h"

//////////////////////////////////////////////////////////////////////////////////
/// \brief slam_monteCarloSearch
///		Function for correcting matching the Laserscan into the map.
/// \param slam
///		Slam container structure containing robot position and lidar data
/// \param sigma_xy
///		spreading of the values around the robot position for trying new positions
/// \param sigma_psi
///		spreading of the values around the robot orientation...
/// \param stop
///		Amount of tries
/// \return
///		Value proportional to the degree of matching
int16_t slam_monteCarloSearch(slam_t *slam, int16_t sigma_xy, int16_t sigma_psi, uint16_t stop)
{
	slam_position_t currentpos; //Stores position with the current spreading
	slam_position_t bestpos; //Stores position with the best matching position
	slam_position_t lastbestpos; //Stores position with the current spreading if a better matching position was found. Used after 1/3 of stop!
	int32_t currentdist; //Stores current value of degree of matching of the laserdata
	int32_t dist_best, lastdist_best; //Stores the best and last best value of degree of matching of the laserdata

	currentpos = bestpos = lastbestpos = slam->robot_pos; //Initialize with robot position
	dist_best = lastdist_best = currentdist = slam_distanceScanToMap(slam, &currentpos); //initialize with current degree of matching

	for(uint16_t i = 0; i < stop; i++)
	{
		currentpos = lastbestpos;
		currentpos.coord.x += (float)((rand() % (sigma_xy * 2)) - sigma_xy); //Generate a random number between -sigma_xy and sigma_xy
		currentpos.coord.y += (float)((rand() % (sigma_xy * 2)) - sigma_xy);
		currentpos.psi += (float)((rand() % (10 * 2)) - 10);
		//currentpos.psi += (float)((rand() % (sigma_psi * 2)) - sigma_psi);

		currentdist = slam_distanceScanToMap(slam, &currentpos); //evaluate this position

		if(currentdist > dist_best) //This position matches better than the best position until now
		{
			dist_best = currentdist; //Overwrite with current match
			bestpos = currentpos; //Overwrite best position
		}

		if((i > (stop / 3)) && (dist_best > lastdist_best)) //Use lastbestpos as start position in every new iteration from now
		{
			lastbestpos = bestpos;
			lastdist_best = dist_best;
			sigma_xy >>= 1; //Division by 2 via bitshifting (very fast): Lower spreading
			sigma_psi >>= 1;
		}
	}

	slam->robot_pos = bestpos;

	return dist_best;
}
