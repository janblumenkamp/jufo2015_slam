#include "slamdefs.h"
#include <math.h>
#include <stdlib.h>
#include "printf.h"

int slam_monteCarloSearch(slam_t *slam, int sigma_xy, int sigma_psi, int stop)
{
	slam_position_t currentpos, bestpos, lastbestpos;
	int currentdist;
	int dist_best, lastdist_best;
	int counter = 0;

	currentpos = bestpos = lastbestpos = slam->robot_pos;
	currentdist = slam_distanceScanToMap(slam, &currentpos);
	dist_best = lastdist_best = currentdist;

	do
	{
		currentpos = lastbestpos;
		currentpos.coord.x += (float)((rand() % (sigma_xy * 2)) - sigma_xy);
		currentpos.coord.y += (float)((rand() % (sigma_xy * 2)) - sigma_xy);
		currentpos.psi += (float)((rand() % (sigma_psi * 2)) - sigma_psi);

		currentdist = slam_distanceScanToMap(slam, &currentpos);

		if ((currentdist > dist_best) && (currentdist != 2000000000))
		{
			dist_best = currentdist;
			bestpos = currentpos;
		}
		else
		{
			counter++;
		}

		if (counter > stop / 3)
		{
			if ((currentdist > dist_best) && (currentdist != 2000000000))
			{
				lastbestpos = bestpos;
				lastdist_best = dist_best;
				counter = 0;
				sigma_xy >>= 1; //Division by 2
				sigma_psi >>= 1;
			}
		}
	} while (counter < stop);

	slam->robot_pos = bestpos;

	return dist_best;
}
