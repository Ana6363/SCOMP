#ifndef COLLISION_H
#define COLLISION_H

#include "shared_memory.h"

void detect_collisions(SharedMemory* shm);
double calculate_dist(SharedDrone d1, SharedDrone d2);
void shutdown_drone(SharedMemory* shm, int drone_index);
bool check_collision_threshold(SharedMemory* shm);

#endif
