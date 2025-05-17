#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "drone.h"

volatile sig_atomic_t drone_down = 0;
static int this_drone_id;

void handle_sigusr1(int sig) {
    char msg[100];
    int len = snprintf(msg, sizeof(msg), "Drone %d received shutdown signal. Ending process.\n", this_drone_id);
    write(STDOUT_FILENO, msg, len);
    _exit(EXIT_SUCCESS);
}



void drone_process(Drone* drone, const char* script_file, int drone_id) {
	this_drone_id = drone_id;
    signal(SIGUSR1, handle_sigusr1);

    close(drone->pipe_read);

    FILE* file = fopen(script_file, "r");
    if (!file) {
        perror("Error opening drone script file");
        exit(EXIT_FAILURE);
    }

    char line[256];
    double t, x, y, z;
    Position pos = {0};

    while (fgets(line, sizeof(line), file) && !drone_down) {
        if (sscanf(line, "%lf %lf %lf %lf", &t, &x, &y, &z) == 4) {
            pos.x = x; pos.y = y; pos.z = z; pos.time = t;
            write(drone->pipe_write, &pos, sizeof(Position));
            usleep((int)(t * 1000000));
        }
    }

    fclose(file);
    close(drone->pipe_write);
}
