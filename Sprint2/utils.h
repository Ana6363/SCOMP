#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include "drone.h"

int count_lines(const char* filename);

double calculate_dist(Drone d1, Drone d2);

#endif
