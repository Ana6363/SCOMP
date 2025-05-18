#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/wait.h>
#include <math.h>
#include <time.h>

#include "drone.h"
#include "utils.h"
#include "report.h"
#include "collision.h"


Drone drones[MAX_DRONES];
int num_drones = 0;
volatile sig_atomic_t simulation_running = 1;
char figure_filename[256] = "";
double radius_collision = 0.0;

void handle_sigint(int sig);
int loading_files(char* filenames[], int count);
void start_simulation();
void detect_collisions();
void shutdown_drone(int index);
double calculate_dist(Drone d1, Drone d2);
void cleaning_drones();


void handle_sigint(int sig) {
    printf("\n[!] Simulation interrupted by user. Cleaning up...\n");
    cleaning_drones();
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <drone_script1.txt> <drone_script2.txt> ...\n", argv[0]);
        return EXIT_FAILURE;
    }

    signal(SIGINT, handle_sigint);

    if (loading_files(&argv[1], argc - 1) != 0) {
        fprintf(stderr, "Error: Could not load drone scripts.\n");
        return EXIT_FAILURE;
    }
	
	
	printf("Enter collision detection radius: ");
	scanf("%lf",&radius_collision);
	 
	
    start_simulation();

    create_report(drones, num_drones, collision_ocurred, collisions, num_collision);

    return EXIT_SUCCESS;
}

int loading_files(char* filenames[], int count) {
    for (int i = 0; i < count; i++) {
        if (num_drones >= MAX_DRONES) break;

        // Copiar nome do ficheiro
        strncpy(drones[num_drones].script_drone, filenames[i], sizeof(drones[num_drones].script_drone) - 1);
        drones[num_drones].script_drone[sizeof(drones[num_drones].script_drone) - 1] = '\0';

        // Extrair o número do ficheiro (ex: drone_7_script.txt → 7)
        int extracted_id = -1;
        sscanf(filenames[i], "drone_%d", &extracted_id);
        if (extracted_id < 0) {
            extracted_id = num_drones; // fallback
        }

        drones[num_drones].id = extracted_id;
        
        drones[num_drones].total_steps = counting_lines(filenames[i]);

        num_drones++;
    }
    return 0;
}



void start_simulation() {
    int pipes[MAX_DRONES][2];

    for (int i = 0; i < num_drones; i++) {
        if (pipe(pipes[i]) < 0) {
            perror("pipe");
            exit(EXIT_FAILURE);
        }

        pid_t pid = fork();
        if (pid < 0) {
            perror("fork");
            exit(EXIT_FAILURE);
        }

        if (pid == 0) {
            // --- CHILD PROCESS ---
            close(pipes[i][0]); // Close read end
            drones[i].pipe_write = pipes[i][1];
            drone_process(&drones[i], drones[i].script_drone, drones[i].id);  
            exit(EXIT_SUCCESS);
        } else {
            drones[i].pid = pid;
            drones[i].pipe_read = pipes[i][0];
            drones[i].active = true;
            close(pipes[i][1]); // Close write end
        }

    }

    int max_steps = counting_lines(drones[0].script_drone);

    for (int step = 0; step < max_steps; step++) {
        for (int i = 0; i < num_drones; i++) {
            if (!drones[i].active) continue;

            Position pos;
            ssize_t r = read(drones[i].pipe_read, &pos, sizeof(Position));

            if (r == sizeof(Position)) {
                drones[i].x = pos.x;
                drones[i].y = pos.y;
                drones[i].z = pos.z;
                drones[i].position_time = pos.time;
                printf("Step %d - Drone %d Position: (%.2f, %.2f, %.2f)\n", step, drones[i].id, pos.x, pos.y, pos.z);
            } else {
                printf("Error: Drone %d received invalid data at step %d\n", drones[i].id, step);
                drones[i].active = false;
                continue;
            }
        }

        detect_collisions();

    }

    cleaning_drones();
}


void shutdown_drone(int index) {
    if (!drones[index].active) return;

    double final_x = drones[index].x;
    double final_y = drones[index].y;
    double final_z = drones[index].z;

    printf("Ending Drone %d with final position: (%.2f, %.2f, %.2f)\n", drones[index].id, final_x, final_y, final_z);

    drones[index].active = false;
    kill(drones[index].pid, SIGUSR1);
    waitpid(drones[index].pid, NULL, 0);

    drones[index].x = final_x;
    drones[index].y = final_y;
    drones[index].z = final_z;

    printf("Drone %d updated position: (%.2f, %.2f, %.2f)\n\n", drones[index].id, drones[index].x, drones[index].y, drones[index].z);
}

void cleaning_drones() {
    for (int i = 0; i < num_drones; i++) {
        if (drones[i].active) {
            shutdown_drone(i);
            printf("Drone %d cleaned up\n\n", drones[i].id);
        }
    }
}


