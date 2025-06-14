#include <stdio.h>
#include <math.h>
#include "utils.h"
#include "drone.h"

int count_lines(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return -1;
    }

    int count = 0;
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        count++;
    }

    fclose(file);
    return count;
}

double calculate_dist(Drone d1, Drone d2) {
    double dx = d1.x - d2.x;
    double dy = d1.y - d2.y;
    double dz = d1.z - d2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}
