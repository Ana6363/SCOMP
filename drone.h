#ifndef DRONE_H
#define DRONE_H

#include <stdbool.h>
#include <sys/types.h>

#define MAX_DRONES 100

typedef struct {
    double x, y, z;
    double time;
} Position;

typedef struct {
    int id;
    double x, y, z;
    pid_t pid;
    int pipe_read;
    int pipe_write;
    bool active;
    char script_drone[256];
    double position_time;
    int total_steps;
} Drone;

void drone_process(Drone* drone, const char* script_file, int drone_id);

#endif
