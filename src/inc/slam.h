#ifndef SLAM_H
#define SLAM_H

#include "CoreSLAM.h"

extern ts_map_t map;

extern void LCD_DispMap(int16_t x0, int16_t y0, int16_t width, int16_t height, ts_map_t *map);

#endif // SLAM_H
