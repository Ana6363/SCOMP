#ifndef COLLISION_H
#define COLLISION_H

#include "drone.h"

#define MAX_COLLISIONS 5

typedef struct {
    int drone1_id, drone2_id;
    double distance, time;
    double dr_x1, dr_y1, dr_z1;
    double dr_x2, dr_y2, dr_z2;
} Collisions;

void detect_collisions(void);

extern Collisions collisions[MAX_COLLISIONS];
extern int num_collision;
extern bool collision_ocurred;
extern double radius_collision;

#endif
