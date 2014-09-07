#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "slamdefs.h"
#include "math.h"
#include "stdlib.h"
#include "printf.h"

/////////////////////////////////////////////////////////////////////////////
/// \brief slam_init
///		Initialisation of the slam container with all relevant information
///		Attention: The constants are not defined here! See slamdefs.h!
/// \param slam
///		Containerstructure. All Infirmation are saved here.
/// \param rob_x_start
///		Robots startposition: x
/// \param rob_y_start
///		Robots startposition: y
/// \param rob_z_start
///		Robots startposition/layer: z
/// \param rob_psi_start
///		Robots startangle
/// \param lidar_val
///		Pointer to the Lidar array of the lenght LASERSCAN_POINTS
/// \param odo_l
///		Pointer to the left odometer variable
/// \param odo_r
///		Pointer to the right odometer variable
///

void slam_init(slam_t *slam,
			   int16_t rob_x_start, int16_t rob_y_start, u8 rob_z_start, int16_t rob_psi_start,
			   XV11_t *xv11, int32_t *odo_l, int32_t *odo_r)
{
	for(u8 z = 0; z < MAP_SIZE_Z_LAYERS; z ++)
		for(u16 y = 0; y < (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM); y++)
			for(u16 x = 0; x < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); x ++)
				slam->map.px[x][y][z] = 127;

	slam->robot_pos.coord.x = rob_x_start;
	slam->robot_pos.coord.y = rob_y_start;
	slam->robot_pos.coord.z = rob_z_start;
	slam->robot_pos.psi = rob_psi_start;

	slam->sensordata.xv11 = xv11;
	slam->sensordata.odo_l = odo_l;
	slam->sensordata.odo_r = odo_r;
}

///////////////////////////////////////////////////////////////////////////////////
/// \brief slam_laserRayToMap
///		Maps one laser ray of the lidar scan to the map. The value is integrated
///		via an alpha-beta-filter. For the better understanding, please read the
///		description of slam_map_update.
/// \param slam
///		SLAM container structure
/// \param x1
/// \param y1
///		Start of the ray
/// \param x2
/// \param y2
///		x/yp + HOLE_WIDTH
/// \param xp
/// \param yp
///		Distance to the beginning of HOLE_WIDTH. The object itself is on:
///		x,yp + ((x,y2 - x,yp)/2)
/// \param value
///		quality of the integration from 0 to 255. 0 doesn’t integrates the
///		ray into the map, 255 integrates it fully into the map, 127 would
///		integrate it to 50% into the map.
/// \param alpha
///		value of the alpha-beta filter from 0 to 255.

void slam_laserRayToMap(slam_t *slam,
						int x1, int y1, int x2, int y2, int xp, int yp,
						int value, int alpha)
{
	int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
	int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
	slam_map_pixel_t *ptr;

	if ((x1 < 0) || (x1 >= (MAP_SIZE_X_MM/MAP_RESOLUTION_MM)) || (y1 < 0) || (y1 >= (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM)))
		return; // Robot is out of map

	x2c = x2; y2c = y2;
	// Clipping
	if (x2c < 0) {
		if (x2c == x1) return;
		y2c += (y2c - y1) * (-x2c) / (x2c - x1);
		x2c = 0;
	}
	if (x2c >= (MAP_SIZE_X_MM/MAP_RESOLUTION_MM)) {
		if (x1 == x2c) return;
		y2c += (y2c - y1) * ((MAP_SIZE_X_MM/MAP_RESOLUTION_MM) - 1 - x2c) / (x2c - x1);
		x2c = (MAP_SIZE_X_MM/MAP_RESOLUTION_MM) - 1;
	}
	if (y2c < 0) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
		y2c = 0;
	}
	if (y2c >= (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM)) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * ((MAP_SIZE_Y_MM/MAP_RESOLUTION_MM) - 1 - y2c) / (y1 - y2c);
		y2c = (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM) - 1;
	}

	dx = abs(x2 - x1); dy = abs(y2 - y1);
	dxc = abs(x2c - x1); dyc = abs(y2c - y1);
	incptrx = (x2 > x1) ? 1 : -1;
	incptry = (y2 > y1) ? (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM) : -(MAP_SIZE_Y_MM/MAP_RESOLUTION_MM);
	sincv = (value > NO_OBSTACLE) ? 1 : -1;
	if (dx > dy) {
		derrorv = abs(xp - x2);
	} else {
		//SWAP(dx, dy); SWAP(dxc, dyc); SWAP(incptrx, incptry);
		dx ^= dy;
		dy ^= dx;
		dx ^= dy;

		dxc ^= dyc;
		dyc ^= dxc;
		dxc ^= dyc;

		incptrx ^= incptry;
		incptry ^= incptrx;
		incptrx ^= incptry;

		derrorv = abs(yp - y2);
	}
	error = 2 * dyc - dxc;
	horiz = 2 * dyc;
	diago = 2 * (dyc - dxc);
	errorv = derrorv / 2;
	incv = (value - NO_OBSTACLE) / derrorv;
	incerrorv = value - NO_OBSTACLE - derrorv * incv;
	ptr = &slam->map.px[0][0][slam->robot_pos.coord.z] + y1 * (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM) + x1;
	pixval = NO_OBSTACLE;
	for (x = 0; x <= dxc; x++, ptr += incptrx) {
		if (x > dx - 2 * derrorv) {
			if (x <= dx - derrorv) {
				pixval += incv;
				errorv += incerrorv;
				if (errorv > derrorv) {
					pixval += sincv;
					errorv -= derrorv;
				}
			} else {
				pixval -= incv;
				errorv -= incerrorv;
				if (errorv < 0) {
					pixval -= sincv;
					errorv += derrorv;
				}
			}
		}
		// Integration into the map
		*ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;
		if (error > 0) {
			ptr += incptry;
			error += diago;
		} else error += horiz;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// \brief slam_map_update
///		Updates one whole scan; integrates one whole scan of the lidar into the map.
/// \param slam
///		SLAM container structure
/// \param quality
///		quality of the integration from 0 to 255. 0 doesn’t integrates the
///		ray into the map, 255 integrates it fully into the map, 127 would
///		integrate it to 50% into the map.
/// \param hole_width
///		width of the hole: Due to the particle filter it is not possible to just
///		save "There is an obstacle or there is no obstacle". We need a grayscale map
///		with some kind of probility of the wall that also somehow represents the
///		accuracy of the lidar. So we say that the probility of the wall gets higher
///		before the actual measured distance of the lidar up to 100% and then goes down
///		again:
///
///    lidar|-----------------------------| obstacle //lidar, the laser ray and an obstacle
/// Probility
/// of obstacle:
/// 0 %     |–––––––––––––––––––––––––\       /––
///         |                          \     /
///         |                           \   /
/// 100 %   |                            \_/
///         ––––––––––––––––––––––––––––––––––––––––> Distance
///									  |-------|
///									  hole_width!!!

void slam_map_update(slam_t *slam, int quality, int hole_width)
{
	float c, s, lidar_x, lidar_y;
	float x2p, y2p;
	float add, dist;
	int x1, y1, x2, y2, xp, yp; //Needed for slam_laserRayToMap

	c = cosf(slam->robot_pos.psi * M_PI / 180);
	s = sinf(slam->robot_pos.psi * M_PI / 180);
	x1 = (int)floorf(slam->robot_pos.coord.x / MAP_RESOLUTION_MM); //Rounds off. Startposition of the lidar/laserray
	y1 = (int)floorf(slam->robot_pos.coord.y / MAP_RESOLUTION_MM);

	for (u16 i = 0; i < 360; i++)
	{
		if(slam->sensordata.xv11->dist_polar[i] != XV11_VAR_NODATA)
		{
			lidar_x = (slam->sensordata.xv11->dist_polar[i] * sinf(i * (M_PI / 180)));
			lidar_y = (slam->sensordata.xv11->dist_polar[i] * cosf(i * (M_PI / 180)));

			x2p = c * lidar_x - s * lidar_y;
			y2p = s * lidar_x + c * lidar_y;

			xp = (int)floorf((slam->robot_pos.coord.x + x2p) / MAP_RESOLUTION_MM);
			yp = (int)floorf((slam->robot_pos.coord.y + y2p) / MAP_RESOLUTION_MM);

			dist = sqrtf(x2p * x2p + y2p * y2p);
			add = hole_width / 2 / dist;
			x2p *= ((1 + add) / MAP_RESOLUTION_MM);
			y2p *= ((1 + add) / MAP_RESOLUTION_MM);

			x2 = (int)floorf(slam->robot_pos.coord.x / MAP_RESOLUTION_MM + x2p);
			y2 = (int)floorf(slam->robot_pos.coord.y / MAP_RESOLUTION_MM + y2p);

			slam_laserRayToMap(slam, y1, x1, y2, x2, yp, xp, IS_OBSTACLE, quality); //Change x/y - otherwise map is mirrored
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////
/// \brief slam_distanceScanToMap
///		comperares the ambiguity  of the laserscan to the given position in the map
/// \param slam
///		slam container structure containing the newest lidar scan
/// \param position
///		position in the map that shall be compared by the lidar scan
/// \return
///		number that is proportional to the ambiguity (around 230000 fully matching)

int slam_distanceScanToMap(slam_t *slam, slam_position_t *position)
{
	float c, s, lidar_x, lidar_y;
	int i, x, y, nb_points = 0, sum = 0;

	c = cosf(position->psi * M_PI / 180);
	s = sinf(position->psi * M_PI / 180);
	// Translate and rotate scan to robot position
	// and compute the distance
	for (i = 0; i < 360; i++)
	{
		if(slam->sensordata.xv11->dist_polar[i] != XV11_VAR_NODATA)
		{
			lidar_x = (slam->sensordata.xv11->dist_polar[i] * sinf(i * (M_PI / 180)));
			lidar_y = (slam->sensordata.xv11->dist_polar[i] * cosf(i * (M_PI / 180)));

			x = (int)floorf((position->coord.x + c * lidar_x - s * lidar_y) / MAP_RESOLUTION_MM);
			y = (int)floorf((position->coord.y + s * lidar_x + c * lidar_y) / MAP_RESOLUTION_MM);
			// Check boundaries
			if ((x >= 0) && (x < (MAP_SIZE_X_MM/MAP_RESOLUTION_MM)) && (y >= 0) && (y < (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM)))
			{
				sum += slam->map.px[x][y][position->coord.z];
				nb_points++;
			}
		}
	}
	if (nb_points) sum = sum * 1024 / nb_points;
	else sum = 2000000000;
	return (int)sum;
}