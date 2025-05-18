#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "report.h"
#include "utils.h"
#include "collision.h"

#define NAME_REPORT "Simulation_Report.txt"

void create_report(Drone drones[], int num_drones, bool collision_ocurred, Collisions collisions[], int num_collisions) {
    FILE* file_report = fopen(NAME_REPORT, "w");
    if (!file_report) {
        perror("Error creating report file");
        return;
    }

    // Header
    time_t current_time = time(NULL);
    char time_str[100];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&current_time));

    fprintf(file_report, "=======================================================\n");
    fprintf(file_report, "             DRONE FIGURE SIMULATION REPORT            \n");
    fprintf(file_report, "=======================================================\n\n");
    fprintf(file_report, "Generated: %s\n", time_str);
    fprintf(file_report, "Figure File: \n\n");

    fprintf(file_report, "SUMMARY\n");
    fprintf(file_report, "-------\n");
    fprintf(file_report, "Total Drones: %d\n", num_drones);
    fprintf(file_report, "Total of Collisions: %d\n", num_collisions);
    fprintf(file_report, "Maximum number of collisions to failed: %d\n", MAX_COLLISIONS);
    fprintf(file_report, "Validation Result: %s\n\n", num_collisions >= MAX_COLLISIONS ? "FAILED" : "PASSED");

    // Drone Details
    fprintf(file_report, "DRONE STATUS\n");
    fprintf(file_report, "------------\n");
    for (int i = 0; i < num_drones; i++) {
        int steps = drones[i].total_steps;


        // Check if this drone was in a collision
        bool involved_in_collision = false;
        for (int j = 0; j < num_collisions; j++) {
            if (collisions[j].drone1_id == drones[i].id || collisions[j].drone2_id == drones[i].id) {
                involved_in_collision = true;
                break;
            }
        }

        const char* status;
        if (drones[i].active) {
            status = "Active";
        } else if (involved_in_collision) {
            status = "Terminated (Collision)";
        } else {
            status = "Terminated (Completed)";
        }

        fprintf(file_report, "Drone %d:\n", drones[i].id);
        fprintf(file_report, "  Script: %s\n", drones[i].script_drone);
        fprintf(file_report, "  Total Steps: %d\n", steps);
        fprintf(file_report, "  Status: %s\n", status);

        if (drones[i].x == 0.0 && drones[i].y == 0.0 && drones[i].z == 0.0 && !drones[i].active) {
            fprintf(file_report, "  Final Position: Unknown (position not saved before termination)\n\n");
        } else {
            fprintf(file_report, "  Final Position: (%.2f, %.2f, %.2f)\n\n", 
                    drones[i].x, drones[i].y, drones[i].z);
        }
    }

    // Collision Info & Recommendations
    if (collision_ocurred) {
        fprintf(file_report, "COLLISION DETAILS\n");
        fprintf(file_report, "------------------\n");
        for (int i = 0; i < num_collisions; i++) {
            fprintf(file_report, "Collision %d:\n", i + 1);
            fprintf(file_report, "  Drones Involved: %d and %d\n", 
                    collisions[i].drone1_id, collisions[i].drone2_id);
            fprintf(file_report, "  Time: %.2f seconds\n", collisions[i].time);        
            fprintf(file_report, "  Distance: %.2f meters\n", collisions[i].distance);
            fprintf(file_report, "  Drone %d Position: (%.2f, %.2f, %.2f)\n", 
                    collisions[i].drone1_id, 
                    collisions[i].dr_x1, collisions[i].dr_y1, collisions[i].dr_z1);
            fprintf(file_report, "  Drone %d Position: (%.2f, %.2f, %.2f)\n\n", 
                    collisions[i].drone2_id, 
                    collisions[i].dr_x2, collisions[i].dr_y2, collisions[i].dr_z2);
        }

        fprintf(file_report, "RECOMMENDATIONS\n");
        fprintf(file_report, "----------------\n");
        fprintf(file_report, "Modify drone paths to prevent the above collisions.\n");
    } else {
        fprintf(file_report, "RECOMMENDATIONS\n");
        fprintf(file_report, "----------------\n");
        fprintf(file_report, "The figure is safe. No collisions detected.\n");
    }

    fclose(file_report);
    printf("Simulation report generated: %s\n", NAME_REPORT);
}
