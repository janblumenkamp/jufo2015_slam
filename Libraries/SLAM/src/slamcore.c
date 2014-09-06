#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "slamdefs.h"

void slam_init(slam_t *slam,
			   int16_t rob_x_start, int16_t rob_y_start, u8 rob_z_start, int16_t rob_psi_start,
			   volatile int16_t *lidar_val, int32_t *odo_l, int32_t *odo_r)
{
	for(u8 z = 0; z < MAP_SIZE_Z_LAYERS; z ++)
		for(u16 y = 0; y < (MAP_SIZE_Y_MM/MAP_RESOLUTION_MM); y++)
			for(u16 x = 0; x < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); x ++)
				slam->map.px[x][y][z] = 0;

	slam->robot_pos.coord.x = rob_x_start;
	slam->robot_pos.coord.y = rob_y_start;
	slam->robot_pos.coord.z = rob_z_start;
	slam->robot_pos.psi = rob_psi_start;

	slam->sensordata.lidarVal = lidar_val;
	slam->sensordata.odo_l = odo_l;
	slam->sensordata.odo_r = odo_r;
}
