#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "slamdefs.h"
#include "math.h"
#include "stdlib.h"
#include "outf.h"
#include "xv11.h"

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
///		Pointer to the left odometer variable. ATTENTION: IF THE VALUE ALREADY INCREASED, IT HAS TO KEEP THE VALUE, OTHERWISE THE ROBOT WILL JUMP IN THE MAP!
/// \param odo_r
///		Pointer to the right odometer variable
///

void slam_init(slam_t *slam,
			   int16_t rob_x_start, int16_t rob_y_start, u_int8_t rob_z_start, int16_t rob_psi_start, int32_t *odo_l, int32_t *odo_r)
{
	for(u8 z = 0; z < MAP_SIZE_Z_LAYERS; z ++)
		for(u16 y = 0; y < (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM); y++)
			for(u16 x = 0; x < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); x ++)
			{
				slam->map.nav[x / MAP_NAVRESOLUTION_FAC][y / MAP_NAVRESOLUTION_FAC][z] = 0;
				slam->map.px[x][y][z] = 127;
			}

	slam->robot_pos.coord.x = rob_x_start;
	slam->robot_pos.coord.y = rob_y_start;
	slam->robot_pos.coord.z = rob_z_start;
	slam->robot_pos.psi = rob_psi_start;

	slam->sensordata.odo_l = odo_l;
	slam->sensordata.odo_r = odo_r;
	slam->sensordata.odo_l_old = *slam->sensordata.odo_l;
	slam->sensordata.odo_r_old = *slam->sensordata.odo_r;
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
	if (dx > dy)
	{
		derrorv = abs(xp - x2);
	}
	else
	{
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
/*
	int16_t index_x = x1;
	int16_t index_y = y1;

	//ptr = &slam->map.px[0][0][slam->robot_pos.coord.z] + y1 * (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) + x1;
	pixval = NO_OBSTACLE;
	for (x = 0; x <= dxc; x++, index_x += incptrx)
	{
		if (x > dx - 2 * derrorv)
		{
			if (x <= dx - derrorv)
			{
				pixval += incv;
				errorv += incerrorv;
				if (errorv > derrorv)
				{
					pixval += sincv;
					errorv -= derrorv;
				}
			}
			else
			{
				pixval -= incv;
				errorv -= incerrorv;
				if (errorv < 0)
				{
					pixval -= sincv;
					errorv += derrorv;
				}
			}
		}
		// Integration into the map
		slam->map.px[index_y][index_x][slam->robot_pos.coord.z] = ((256 - alpha) * (slam->map.px[index_y][index_x][slam->robot_pos.coord.z]) + alpha * pixval) >> 8;

		if (error > 0)
		{
			index_y += incptry;

			error += diago;
		}
		else
			error += horiz;
	}*/

	ptr = &slam->map.px[0][0][slam->robot_pos.coord.z] + y1 * (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) + x1;
	pixval = NO_OBSTACLE;
	for (x = 0; x <= dxc; x++, ptr += incptrx)
	{
		if (x > dx - 2 * derrorv)
		{
			if (x <= dx - derrorv)
			{
				pixval += incv;
				errorv += incerrorv;
				if (errorv > derrorv)
				{
					pixval += sincv;
					errorv -= derrorv;
				}
			}
			else
			{
				pixval -= incv;
				errorv -= incerrorv;
				if (errorv < 0)
				{
					pixval -= sincv;
					errorv += derrorv;
				}
			}
		}
		// Integration into the map
		*ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;
		if (error > 0)
		{
			ptr += incptry;
			error += diago;
		}
		else error += horiz;
	}
}


void slam_laserRayToNav(slam_t *slam,
						int x1, int y1, int x2, int y2, int xp, int yp,
						int value, int alpha)
{
	int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
	int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
	slam_map_navpixel_t *ptr;

	if ((x1 < 0) || (x1 >= MAP_NAV_SIZE_X_PX) || (y1 < 0) || (y1 >= MAP_NAV_SIZE_Y_PX))
		return; // Robot is out of map

	x2c = x2; y2c = y2;
	// Clipping
	if (x2c < 0) {
		if (x2c == x1) return;
		y2c += (y2c - y1) * (-x2c) / (x2c - x1);
		x2c = 0;
	}
	if (x2c >= MAP_NAV_SIZE_X_PX) {
		if (x1 == x2c) return;
		y2c += (y2c - y1) * (MAP_NAV_SIZE_X_PX - 1 - x2c) / (x2c - x1);
		x2c = MAP_NAV_SIZE_X_PX - 1;
	}
	if (y2c < 0) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
		y2c = 0;
	}
	if (y2c >= MAP_NAV_SIZE_Y_PX) {
		if (y1 == y2c) return;
		x2c += (x1 - x2c) * (MAP_NAV_SIZE_Y_PX - 1 - y2c) / (y1 - y2c);
		y2c = MAP_NAV_SIZE_Y_PX - 1;
	}

	dx = abs(x2 - x1); dy = abs(y2 - y1);
	dxc = abs(x2c - x1); dyc = abs(y2c - y1);
	incptrx = (x2 > x1) ? 1 : -1;
	incptry = (y2 > y1) ? MAP_NAV_SIZE_Y_PX : -MAP_NAV_SIZE_Y_PX;
	sincv = (value > NO_OBSTACLE) ? 1 : -1;
	if (dx > dy)
	{
		derrorv = abs(xp - x2);
	}
	else
	{
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

	ptr = &slam->map.nav[0][0][slam->robot_pos.coord.z] + y1 * MAP_NAV_SIZE_Y_PX + x1;
	pixval = NO_OBSTACLE;
	for (x = 0; x <= dxc; x++, ptr += incptrx)
	{
		if (x > dx - 2 * derrorv)
		{
			if (x <= dx - derrorv)
			{
				pixval = IS_OBSTACLE;//pixval += incv;
				errorv += incerrorv;
				if (errorv > derrorv)
				{
					pixval += sincv;
					errorv -= derrorv;
				}
			}
			else
			{
				pixval = IS_OBSTACLE;//pixval -= incv;
				errorv -= incerrorv;
				if (errorv < 0)
				{
					//pixval -= sincv;
					errorv += derrorv;
				}
			}
		}
		// Integration into the map
		*ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;
		if (error > 0)
		{
			ptr += incptry;
			error += diago;
		}
		else error += horiz;
	}
}

///////////////////////////////////////////////////////////////
/// \brief Maps one laser ray of the lidar scan to the map. The value is integrated
///		via an alpha-beta-filter. For the better understanding, please read the
///		description of slam_map_update.
/// \param slam
///		SLAM container structure
/// \param xs
///		X root of the line
/// \param ys
///		Y root
/// \param xe
///		X End of line (end of laser ray)
/// \param ye
///		Y End
/// \param xh
///		X beginning of HOLE_WIDTH (Value now decreases)
/// \param yh
///		Y HOLE_WIDTH
/// \param updateRate
///		How fast is the line integrated?
/// \param value
///
void slam_line(slam_t *slam, int xs, int ys, int xe, int ye, int xh, int yh, uint8_t updateRate)
{
	//Source: http://de.wikipedia.org/wiki/Bresenham-Algorithmus#Kompakte_Variante
/*	int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = dx + dy, e2; // error value e_xy

	int value = 0;

	while(x0 != x1 || y0 != y1) //Calculate amount of pixels in line
	{
		slam->map.px[x0][y0][slam->robot_pos.coord.z] = ((256 - updateRate) * (slam->map.px[x0][y0][slam->robot_pos.coord.z]) + updateRate * value) >> 8;

		e2 = 2 * err;
		if(e2 > dy)  // e_xy + e_x > 0
		{
			err += dy;
			x0 += sx;
		}
		if(e2 < dx) // e_xy + e_y < 0
		{
			err += dx;
			y0 += sy;
		}
	}*/
}

////////////////////////////////////////////////////////////////////////////////
/// \brief slam_map_update
///		Updates one whole scan; integrates one whole scan of the lidar into the map.
/// \param slam
///		SLAM container structure
/// \param map
///		Update raw map or the navigation area
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

void slam_map_update(slam_t *slam, u8 map, int quality, int hole_width)
{
	float c, s;
	float x2p, y2p;
	int i, x1, y1, x2, y2, xp, yp;
	float add, dist;

	float lidar_x, lidar_y;

	c = cosf((slam->robot_pos.psi) * M_PI / 180);
	s = sinf((slam->robot_pos.psi) * M_PI / 180);
	x1 = (int)floorf(slam->robot_pos.coord.y / MAP_RESOLUTION_MM + 0.5); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	y1 = (int)floorf((slam->robot_pos.coord.x) / MAP_RESOLUTION_MM + 0.5);
	// Translate and rotate scan to robot position
	for (i = 0; i < LASERSCAN_POINTS; i++)
	{
		if(slam->sensordata.lidar[i] != LASERSCAN_NODATA)
		{
			lidar_x = (slam->sensordata.lidar[i] * sinf(i * (M_PI / 180)));
			lidar_y = (slam->sensordata.lidar[i] * cosf(i * (M_PI / 180)));

			x2p = c * lidar_x - s * lidar_y;
			y2p = s * lidar_x + c * lidar_y;

			xp = (int)floorf((slam->robot_pos.coord.y + x2p) / MAP_RESOLUTION_MM + 0.5);//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			yp = (int)floorf((slam->robot_pos.coord.x + y2p) / MAP_RESOLUTION_MM + 0.5);

			dist = sqrtf(x2p * x2p + y2p * y2p);
			add = hole_width / 2 / dist;
			x2p = x2p / MAP_RESOLUTION_MM * (1 + add);
			y2p = y2p / MAP_RESOLUTION_MM * (1 + add);

			x2 = (int)floorf(slam->robot_pos.coord.y / MAP_RESOLUTION_MM + x2p + 0.5);//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			y2 = (int)floorf(slam->robot_pos.coord.x / MAP_RESOLUTION_MM + y2p + 0.5);

			if(map)	slam_laserRayToMap(slam, x1, y1, x2, y2, xp, yp, IS_OBSTACLE, quality);
			else	slam_laserRayToNav(slam, x1/3, y1/3, x2/3, y2/3, xp/3, yp/3, IS_OBSTACLE, quality);
		}
	}
	//for(int i = 0; i < MAP_SIZE_X_MM / (MAP_RESOLUTION_MM * 3); i++)
	//	slam->map.nav[i][i][0] = i;
}

////////////////////////////////////////////////////////////////////////////////////
/// \brief slam_distanceScanToMap
///		Matches the Laserscan on the given position in the map
/// \param slam
///		slam container structure containing the newest lidar scan
/// \param position
///		position in the map that shall be compared by the lidar scan
/// \return
///		number that is proportional to the ambiguity (around 230000 fully matching),
///		-1 if no match found

int slam_distanceScanToMap(slam_t *slam, slam_position_t *position)
{
	float c, s, lidar_x, lidar_y;
	int i, x, y, nb_points = 0;
	float sum = 0;

	c = cosf((position->psi) * M_PI / 180); //Calculate it here, not nessesary to calculate in every iteration in the loop
	s = sinf((position->psi) * M_PI / 180);
	// Translate and rotate scan to robot position
	// and compute the distance
	for (i = 0; i < LASERSCAN_POINTS; i += 10) //LASERSCAN_POINTS: 360. For every 10th measurement.
	{
		if(slam->sensordata.lidar[i] != LASERSCAN_NODATA) //If the quality of the measurement is high enough and not out of range
		{
			lidar_x = (slam->sensordata.lidar[i] * sinf(i * (M_PI / 180))); //Convert from polar to cartesian
			lidar_y = (slam->sensordata.lidar[i] * cosf(i * (M_PI / 180)));

			x = (int)floorf((position->coord.y + c * lidar_x - s * lidar_y) / MAP_RESOLUTION_MM + 0.5); //Calculate the point in which the Measurement ends as seen from the robot.
			y = (int)floorf((position->coord.x + s * lidar_x + c * lidar_y) / MAP_RESOLUTION_MM + 0.5); //Workaround: y- and y- position has to be changed due to strange mirroring error...

			if((x >= 0) && (x < (MAP_SIZE_X_MM/MAP_RESOLUTION_MM)) && (y >= 0) && (y < (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM))) //Point lies inside the map size!
			{
				sum += *(&slam->map.px[0][0][slam->robot_pos.coord.z] + y * (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) + x); //Access array by pointer-arithemtics, add value to sum
				nb_points++;
			}
		}
	}
	if (nb_points) sum = sum * 1024 / nb_points; //Calculate all-in-all value for returning
	else sum = -1;
	return (int)sum;
}

//////////////////////////////////////////////////////////////////////////////////////////
/// \brief slam_processMovement
///		Transfers the driven encoder distance to a cartesian posisition and adds it to the
///		old robot position in the slam structure.
/// \param slam
///		slam container structure
/// \param mot
///		motor information structure
///
/// Source:
/// http://www6.in.tum.de/Main/Publications/5224223.pdf

void slam_processMovement(slam_t *slam)
{
	float dl_enc, dr_enc; //Driven distance (since last function call) in mm.
	float dx = 0, dy = 0, dpsi = 0, dist_driven = 0;

	dl_enc = (*slam->sensordata.odo_l - slam->sensordata.odo_l_old) * 2 * WHEELRADIUS * M_PI / TICKSPERREV; //Calculate difference driven distance in mm
	dr_enc = (*slam->sensordata.odo_r - slam->sensordata.odo_r_old) * 2 * WHEELRADIUS * M_PI / TICKSPERREV;
	slam->sensordata.odo_l_old = *slam->sensordata.odo_l; //Nessesary for next interation (difference)
	slam->sensordata.odo_r_old = *slam->sensordata.odo_r;

	if(fabsf(dl_enc - dr_enc) > 0) //If robot has driven a curve
	{
		float r = -WHEELDIST * (dl_enc + dr_enc) / (2 * (dr_enc - dl_enc)); //Helpervariable
		dpsi = -(dr_enc - dl_enc) / WHEELDIST; //Calculate change of rotation (orientation) of robot

		dx = r * sinf(dpsi + (slam->robot_pos.psi * 180 / M_PI)) - r * sinf((slam->robot_pos.psi * 180 / M_PI)); //Calculate change of cartesian x in relation to last robot position
		dy = -r * cosf(dpsi + (slam->robot_pos.psi * 180 / M_PI)) + r * cosf((slam->robot_pos.psi * 180 / M_PI)); //...y

		dpsi *= 180 / M_PI; //Convert radian to degree
	}
	else // basically going straight
	{
		dx = dl_enc * cosf(slam->robot_pos.psi * M_PI / 180); //No change of rotation
		dy = dr_enc * sinf(slam->robot_pos.psi * M_PI / 180);
	}

	dist_driven = sqrtf(dx * dx + dy * dy); //Driven distance

	slam->robot_pos.coord.x += dist_driven * cosf((180 - slam->robot_pos.psi + dpsi) * M_PI / 180); //The calculated values were given as change of position in relation to the last robot position. Now we have to calculate the new absolute position.
	slam->robot_pos.coord.y += dist_driven * sinf((180 - slam->robot_pos.psi + dpsi) * M_PI / 180);
	slam->robot_pos.psi += dpsi;
}
