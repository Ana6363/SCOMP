#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include "collision.h"
#include "shared_memory.h"

void detect_collisions(SharedMemory* shm) {
    bool collision_detected_this_scan = false;
    
    for (int i = 0; i < shm->num_drones; i++) {
        if (!shm->drones[i].active) continue;

        for (int j = i + 1; j < shm->num_drones; j++) {
            if (!shm->drones[j].active) continue;

            double dist = calculate_dist(shm->drones[i], shm->drones[j]);

            printf("Checking collision between Drone %d and Drone %d. Distance: %.2f\n",
                   shm->drones[i].id, shm->drones[j].id, dist);
            printf("\n");

            if (dist < shm->radius_collision) {
                shm->collision_occurred = true;
                collision_detected_this_scan = true;

                if (shm->num_collisions < MAX_COLLISIONS) {
                    Collisions c = {
                        .drone1_id = shm->drones[i].id,
                        .drone2_id = shm->drones[j].id,
                        .distance = dist,
                        .time = shm->drones[i].position_time,
                        .dr_x1 = shm->drones[i].x, .dr_y1 = shm->drones[i].y, .dr_z1 = shm->drones[i].z,
                        .dr_x2 = shm->drones[j].x, .dr_y2 = shm->drones[j].y, .dr_z2 = shm->drones[j].z
                    };
                    shm->collisions[shm->num_collisions++] = c;

                    printf("[!] COLLISION DETECTED between Drone %d and Drone %d. Distance: %.2f\n",
                           shm->drones[i].id, shm->drones[j].id, dist);
                    printf("[!] Collision count: %d/%d\n", shm->num_collisions, MAX_COLLISIONS);

                    printf("[!] Collision recorded but drones will continue operating\n");
                    printf("\n");
                    
                    if (shm->num_collisions >= MAX_COLLISIONS) {
                        fprintf(stderr, "[X] Maximum number of collisions (%d) reached. Terminating simulation.\n", MAX_COLLISIONS);
                        printf("\n");
                        for (int k = 0; k < shm->num_drones; k++) {
                            if (shm->drones[k].active) {
                                shutdown_drone(shm, k);
                            }
                        }
                        shm->simulation_running = false;
                    } else {
                        printf("[!] Continuing simulation (%d/%d collisions)\n", shm->num_collisions, MAX_COLLISIONS);
                        printf("\n");
                    }
                } else {
                    fprintf(stderr, "[X] Maximum number of collisions (%d) reached. Terminating simulation.\n", MAX_COLLISIONS);
                    printf("\n");
                    shm->simulation_running = false;
                }
            }
        }
    }
    
    if (collision_detected_this_scan) {
        shm->new_collision_detected = true;
    }
}

double calculate_dist(SharedDrone d1, SharedDrone d2) {
    double dx = d1.x - d2.x;
    double dy = d1.y - d2.y;
    double dz = d1.z - d2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void shutdown_drone(SharedMemory* shm, int drone_index) {
    if (!shm->drones[drone_index].active) return;

    double final_x = shm->drones[drone_index].x;
    double final_y = shm->drones[drone_index].y;
    double final_z = shm->drones[drone_index].z;

    printf("Terminating Drone %d with final position: (%.2f, %.2f, %.2f)\n", 
           shm->drones[drone_index].id, final_x, final_y, final_z);
    printf("\n");

    shm->drones[drone_index].active = false;

    shm->drones[drone_index].x = final_x;
    shm->drones[drone_index].y = final_y;
    shm->drones[drone_index].z = final_z;

    printf("Drone %d final position saved: (%.2f, %.2f, %.2f)\n\n", 
           shm->drones[drone_index].id, shm->drones[drone_index].x, 
           shm->drones[drone_index].y, shm->drones[drone_index].z);
    printf("\n");
}

bool check_collision_threshold(SharedMemory* shm) {
    return shm->num_collisions >= MAX_COLLISIONS;
}
