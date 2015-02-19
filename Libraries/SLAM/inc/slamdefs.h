////////////////////////////////////////////////////////////////////////////////
/// slamdefs.h
///
/// Definitions of Data structures etc. of the slam library. All
/// units in mm an degree. The initialization
/// void fills all important data into the structs.
////////////////////////////////////////////////////////////////////////////////

#ifndef SLAMDEFS_H
#define SLAMDEFS_H

#include "main.h"
#include "stm32f4xx.h"

//Sensordata
#define WHEELDIST	260 //Distance between two wheels
#define WHEELRADIUS	26
#define TICKSPERREV	360

#define LASERSCAN_ANGLE		360 //degree
#define LASERSCAN_POINTS	360 //Amount of scans per LASERSCAN_ANGLE ->_POINTS/_ANGLE = (HAS TO BE A NATURAL NUMBER!!!!) resolution of Laserscanner
#define LASERSCAN_NODATA	0 //Var of Laserscan if no data available

#define ODOMETER_TICKS_PER_REV	360 //Odometer ticks per revolution
#define WHEEL_RADIUS			26 //In mm

//Map
#define MAP_SIZE_X_MM			6000
#define MAP_SIZE_Y_MM			6000
#define MAP_SIZE_Z_LAYERS		1		//Amount of layers of the map
#define MAP_RESOLUTION_MM		20
#define MAP_NAVRESOLUTION_FAC	3 //Resolution of navigation cells in MAP_RESOLUTION_MM * MAP_NAVRESOLUTION_FAC mm (on each navresolution cell there come MAP_NAVRESOLUTION_FAC^2 MAP_SIZE_X_MM / MAP_RESOLUTION_MM cells)
#define MAP_NAV_SIZE_X_PX		MAP_SIZE_X_MM / (MAP_RESOLUTION_MM * MAP_NAVRESOLUTION_FAC)
#define MAP_NAV_SIZE_Y_PX		MAP_SIZE_Y_MM / (MAP_RESOLUTION_MM * MAP_NAVRESOLUTION_FAC)

#define MAP_VAR_MAX			255 //Overflow of map pixel
#define MAP_VAR_MIN			0 //Underflow of map pixel

#define IS_OBSTACLE			255 //Obstacle with 100% certainty
#define NO_OBSTACLE			0 //Obstacle with 0% certainty

//Coordinates: location in room (x, y, z)
typedef struct {
	float x;
	float y;
	int8_t z;
} slam_coordinates_t;

// Position: coordinates and orientation (angle (psi))
typedef struct {
	slam_coordinates_t coord;
	float psi;
} slam_position_t;

//Datastruct: (Pointer to) all relevant sensor/hardware information of the robot
typedef struct {
	int32_t *odo_l; //Odometer left
	int32_t *odo_r; //Odometer right
	int32_t odo_l_old; //Last odometer value after call of slam_processMovement
	int32_t odo_r_old;	//"
	int16_t lidar[LASERSCAN_POINTS]; //Laserscan data
} slam_sensordata_t;

typedef u_int8_t slam_map_pixel_t;
typedef u_int8_t slam_map_navpixel_t;

//Raw Map
typedef struct {
	slam_map_pixel_t px[MAP_SIZE_X_MM / MAP_RESOLUTION_MM][MAP_SIZE_Y_MM / MAP_RESOLUTION_MM][MAP_SIZE_Z_LAYERS];
	slam_map_navpixel_t nav[MAP_NAV_SIZE_X_PX][MAP_NAV_SIZE_X_PX][MAP_SIZE_Z_LAYERS];
} slam_map_t;

//Container of all SLAM information:
typedef struct {
	slam_position_t robot_pos;
	slam_sensordata_t sensordata;
	slam_map_t map;
} slam_t;

extern int16_t slam_monteCarloSearch(slam_t *slam, int16_t sigma_xy, int16_t sigma_psi, uint16_t stop);

//Initialization of all relevant SLAM information
extern void slam_init(slam_t *slam,
					  int16_t rob_x_start, int16_t rob_y_start, u_int8_t rob_z_start, int16_t rob_psi_start,
					  int32_t *odo_l, int32_t *odo_r);

//Bases on ts_map_laser_ray
extern void slam_laserRayToMap(slam_t *slam,
							   int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t xp, int16_t yp,
							   int16_t value, int16_t alpha);

extern void slam_laserRayToNav(slam_t *slam,
						int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t xp, int16_t yp,
						int16_t value, int16_t alpha);

extern void slam_map_update(slam_t *slam, u8 map, int16_t quality, int16_t hole_width);

extern int32_t slam_distanceScanToMap(slam_t *slam, slam_position_t *position);

extern void slam_processMovement(slam_t *slam);

extern void slam_line(slam_t *slam, int x0, int y0, int x1, int y1, int xh, int yh, uint8_t updateRate);

#endif
