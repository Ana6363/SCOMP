#ifndef REPORT_H
#define REPORT_H

#include "drone.h"
#include "collision.h"

extern char figure_filename[256];

void generate_report(Drone drones[], int num_drones, bool collision_ocurred, Collisions collisions[], int num_collisions);

#endif
