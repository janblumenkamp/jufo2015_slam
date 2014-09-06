#ifndef SLAM_H
#define SLAM_H

#include "slamdefs.h"

extern slam_t slam;

extern void slam_LCD_DispMap(int16_t x0, int16_t y0, slam_t *slam);

#endif // SLAM_H
