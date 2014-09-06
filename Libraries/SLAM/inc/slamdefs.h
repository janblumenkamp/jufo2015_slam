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

//Sensordata

#define LASERSCAN_ANGLE		360 //degree
#define LASERSCAN_POINTS	360 //Amount of scans per LASERSCAN_ANGLE ->_POINTS/_ANGLE = resolution of Laserscanner

#define ODOMETER_TICKS_PER_REV	360 //Odometer ticks per revolution
#define WHEEL_RADIUS			20

//Map
#define MAP_SIZE_X_MM			2000
#define MAP_SIZE_Y_MM			2000
#define MAP_SIZE_Z_LAYERS		1		//Amount of layers of the map
#define MAP_RESOLUTION_MM		10

#define IS_OBSTACLE			127 //Overflow of map pixel
#define NO_OBSTACLE			-127 //Underflow of map pixel

//Position: location in room (x, y, z) and orientation (angle (psi))
typedef struct {
	int16_t x;
	int16_t y;
	int8_t z;
} slam_coordinates_t;

typedef struct {
	slam_coordinates_t coord;
	int16_t psi;
} slam_position_t;

//Datastruct: (Pointer to) all relevant sensor/hardware information of the robot
typedef struct {
	int32_t *odo_l; //Odometer left
	int32_t *odo_r; //Odometer right
	volatile int16_t *lidarVal; //Pointer to the raw laserscan array
} slam_sensordata_t;

typedef struct {
	int8_t px[MAP_SIZE_X_MM / MAP_RESOLUTION_MM][MAP_SIZE_Y_MM / MAP_RESOLUTION_MM][MAP_SIZE_Z_LAYERS];
} slam_map_t;

//Container of all SLAM information:
typedef struct {
	slam_position_t robot_pos;
	slam_sensordata_t sensordata;
	slam_map_t map;
} slam_t;


void slam_init(slam_t *slam,
			   int16_t rob_x_start, int16_t rob_y_start, u8 rob_z_start, int16_t rob_psi_start,
			   volatile int16_t *lidar_val, int32_t *odo_l, int32_t *odo_r);

#endif
