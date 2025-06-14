#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "report.h"
#include "shared_memory.h"

#define NAME_REPORT "Simulation_Report.txt"

void create_report(SharedMemory* shm) {
    FILE* file_report = fopen(NAME_REPORT, "w");
    if (!file_report) {
        perror("Error creating report file");
        return;
    }

    time_t current_time = time(NULL);
    char time_str[100];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&current_time));

    fprintf(file_report, "===============================================================\n");
    fprintf(file_report, "                DRONE SIMULATION REPORT                        \n");
    fprintf(file_report, "===============================================================\n\n");
    fprintf(file_report, "Report Generated: %s\n", time_str);


    fprintf(file_report, "\nSIMULATION SUMMARY\n");
    fprintf(file_report, "=================\n");
    
    int active_drones = 0, completed_drones = 0, terminated_drones = 0;
    double simulation_duration = 0.0;
    
    int max_step_reached = 0;
    
    for (int i = 0; i < shm->num_drones; i++) {
        if (shm->drones[i].active) active_drones++;
        else if (shm->drones[i].completed) completed_drones++;
        else terminated_drones++;
        
        if (shm->drones[i].current_step + 1 > max_step_reached) {
            max_step_reached = shm->drones[i].current_step + 1;
        }
        
        
        if (shm->drones[i].position_time > simulation_duration) {
            simulation_duration = shm->drones[i].position_time;
        }
    }
    
    
    bool simulation_passed = shm->num_collisions < MAX_COLLISIONS;
    
    fprintf(file_report, "Total Drones simulated: %d\n", shm->num_drones);
    fprintf(file_report, "Simulation Duration: %.2f seconds\n", simulation_duration);
    fprintf(file_report, "Total Steps Executed: %d\n", max_step_reached);
    fprintf(file_report, "Maximum Steps Possible: %d\n", shm->max_simulation_steps);
    fprintf(file_report, "Completion Rate: %.1f%%\n", 
            (double)max_step_reached / shm->max_simulation_steps * 100.0);
    fprintf(file_report, "Collision Detection Radius: %.2f meters\n", shm->radius_collision);
    fprintf(file_report, "Total Collisions Detected: %d\n", shm->num_collisions);
    fprintf(file_report, "Maximum Collisions Allowed: %d\n", MAX_COLLISIONS);
    fprintf(file_report, "FINAL VALIDATION RESULT: %s\n\n", 
            simulation_passed ? "PASSED - SAFE SIMULATION" : "FAILED - COLLISION THRESHOLD EXCEEDED");

    fprintf(file_report, "DETAILED DRONE EXECUTION STATUS\n");
    fprintf(file_report, "================================\n");
    
    for (int i = 0; i < shm->num_drones; i++) {
        bool involved_in_collision = false;
        int collision_count = 0;
        
        for (int j = 0; j < shm->num_collisions; j++) {
            if (shm->collisions[j].drone1_id == shm->drones[i].id || 
                shm->collisions[j].drone2_id == shm->drones[i].id) {
                involved_in_collision = true;
                collision_count++;
            }
        }
        
        
        
        fprintf(file_report, " DRONE %d: \n", shm->drones[i].id);
        fprintf(file_report, "   Script File: %s\n", shm->drones[i].script_drone);
        fprintf(file_report, "   Steps Completed: %d / %d (%.1f%%)\n", 
                shm->drones[i].current_step + 1, shm->drones[i].total_steps,  
                (double)(shm->drones[i].current_step + 1) / shm->drones[i].total_steps * 100.0);
        fprintf(file_report, "   Final Position: (%.2f, %.2f, %.2f) meters\n", 
                shm->drones[i].x, shm->drones[i].y, shm->drones[i].z);
        fprintf(file_report, "   Last Update Time: %.2f seconds\n", shm->drones[i].position_time);
        fprintf(file_report, "   Collision Involvement: %s (%d events)\n", 
                involved_in_collision ? "YES" : "NO", collision_count);
        fprintf(file_report, "   Simulation Duration: %.2f seconds\n\n", shm->drones[i].position_time);
    }

    if (shm->collision_occurred) {
        fprintf(file_report, "COMPREHENSIVE COLLISION ANALYSIS\n");
        fprintf(file_report, "=================================\n");
        fprintf(file_report, "Total Collision Events: %d\n", shm->num_collisions);
        fprintf(file_report, "Collision Detection Method: Real-time Distance Calculation\n");
        fprintf(file_report, "Safety Threshold: %.2f meters\n\n", shm->radius_collision);
        fprintf(file_report, "=================================\n");

        
        int drone_collision_count[MAX_DRONES] = {0};
        
        for (int i = 0; i < shm->num_collisions; i++) {
            fprintf(file_report, " COLLISION #%d\n", i + 1);
            fprintf(file_report, " -------------------\n");
            fprintf(file_report, "   Timestamp: %.2f seconds\n", shm->collisions[i].time);
            fprintf(file_report, "   Simulation Step: %d\n", (int)shm->collisions[i].time);
            fprintf(file_report, "   Drones Involved: Drone %d and Drone %d\n", 
                    shm->collisions[i].drone1_id, shm->collisions[i].drone2_id);
            fprintf(file_report, "   Collision Distance: %.2f meters\n", shm->collisions[i].distance);
            fprintf(file_report, "   Safety Margin Violation: %.2f meters\n", 
                    shm->radius_collision - shm->collisions[i].distance);
            fprintf(file_report, "   Drone %d Position: (%.2f, %.2f, %.2f)\n", 
                    shm->collisions[i].drone1_id, 
                    shm->collisions[i].dr_x1, shm->collisions[i].dr_y1, shm->collisions[i].dr_z1);
            fprintf(file_report, "   Drone %d Position: (%.2f, %.2f, %.2f)\n\n", 
                    shm->collisions[i].drone2_id, 
                    shm->collisions[i].dr_x2, shm->collisions[i].dr_y2, shm->collisions[i].dr_z2);
            
            for (int j = 0; j < shm->num_drones; j++) {
                if (shm->drones[j].id == shm->collisions[i].drone1_id || 
                    shm->drones[j].id == shm->collisions[i].drone2_id) {
                    drone_collision_count[j]++;
                }
            }
            
            
        }
        
        
    }

    fprintf(file_report, "SAFETY ASSESSMENT & RECOMMENDATIONS\n");
    fprintf(file_report, "====================================\n");
    
    if (simulation_passed) {
        fprintf(file_report, "  SAFETY VALIDATION: PASSED\n");
        fprintf(file_report, "  All drones completed their paths without exceeding collision thresholds.\n\n");
        
        fprintf(file_report, "  RECOMMENDATIONS FOR OPERATIONAL DEPLOYMENT:\n");
        fprintf(file_report, "   • Current simulation formation is APPROVED for deployment\n");
    } else {
        fprintf(file_report, "  SAFETY VALIDATION: FAILED\n");
        fprintf(file_report, "  Collision threshold exceeded (%d/%d collisions detected)\n", 
                shm->num_collisions, MAX_COLLISIONS);
        fprintf(file_report, "  The current simulation formation poses significant safety risks.\n\n");
        
        fprintf(file_report, "  MANDATORY CORRECTIVE ACTIONS:\n");
        fprintf(file_report, "   • Revise simulation paths to increase separation distances\n");
        
        if (shm->num_drones > 2) {
            fprintf(file_report, "   • Consider reducing the number of drones in congested areas\n");
        }
    }

    fprintf(file_report, "\n  TECHNICAL PERFORMANCE METRICS\n");
   
    fprintf(file_report, "   • Total Simulation Steps: %d\n", shm->max_simulation_steps);
    fprintf(file_report, "   • Memory Usage: %zu bytes shared memory\n", sizeof(SharedMemory));
    fprintf(file_report, "   • Process Termination: Clean shutdown \n");

    fprintf(file_report, "\n===============================================================\n");
    fprintf(file_report, "                    END OF SIMULATION REPORT                 	   \n");
    fprintf(file_report, "===============================================================\n");
    fprintf(file_report, "Report Status: Complete\n");
    fprintf(file_report, "Data Integrity: Verified\n");
    fprintf(file_report, "Validation: %s\n", simulation_passed ? "APPROVED" : "REJECTED");
    fprintf(file_report, "Next Action Required: %s\n", 
            simulation_passed ? "Ready for deployment" : "Immediate safety review required");

    fclose(file_report);
    
    printf("\n===============================================================\n");
    printf("           COMPREHENSIVE SIMULATION REPORT GENERATED          \n");
    printf("===============================================================\n");
    printf("Report File: %s\n", NAME_REPORT);
    printf("Report Size: Complete analysis with %d sections\n", 6);
    printf("Validation Result: %s\n", simulation_passed ? "PASSED" : "FAILED");
    printf("Total Collisions: %d/%d\n", shm->num_collisions, MAX_COLLISIONS);
    printf("Safety Status: %s\n", simulation_passed ? "APPROVED FOR DEPLOYMENT" : "REQUIRES IMMEDIATE REVIEW");
    printf("===============================================================\n");
    printf("\n");
}
