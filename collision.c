#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <time.h>
#include <stdlib.h>
#include "utils.h"
#include "collision.h"

extern Drone drones[];
extern int num_drones;

extern void shutdown_drone(int index);

Collisions collisions[MAX_COLLISIONS];
int num_collision = 0;
bool collision_ocurred = false;

void detect_collisions() {
	
    for (int i = 0; i < num_drones; i++) {
        if (!drones[i].active) continue;

        for (int j = i + 1; j < num_drones; j++) {
            if (!drones[j].active) continue;

            double dist = calculate_dist(drones[i], drones[j]);

            printf("Checking collision between Drone %d and Drone %d. Distance: %.2f\n\n",
                   drones[i].id, drones[j].id, dist);

            if (dist < radius_collision) {
                collision_ocurred = true;

                if (num_collision < MAX_COLLISIONS) {
                    Collisions c = {
                        .drone1_id = drones[i].id,
                        .drone2_id = drones[j].id,
                        .distance = dist,
                        .time = drones[i].position_time,
                        .dr_x1 = drones[i].x, .dr_y1 = drones[i].y, .dr_z1 = drones[i].z,
                        .dr_x2 = drones[j].x, .dr_y2 = drones[j].y, .dr_z2 = drones[j].z
                    };
                    collisions[num_collision++] = c;

                    printf("[!] Collision detected between Drone %d and Drone %d. Distance: %.2f\n\n",
                           drones[i].id, drones[j].id, dist);
                           continue;
                
                } else {
					
                    fprintf(stderr, "[X] Maximum number of collisions (%d) reached. Terminating simulation.\n\n", MAX_COLLISIONS);
                    
                   
                }

                shutdown_drone(i);
                shutdown_drone(j);
            }
        }
    }
}
