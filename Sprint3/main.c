#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/wait.h>
#include <math.h>
#include <time.h>

#include "shared_memory.h"
#include "thread_manager.h"
#include "drone.h"
#include "utils.h"
#include "report.h"
#include "collision.h"

volatile sig_atomic_t simulation_interrupted = 0;
ThreadManager thread_manager = {0};
pid_t drone_pids[MAX_DRONES];
int num_drone_processes = 0;
bool error_already_handled = false;

void handle_sigint(int sig);
int loading_files(char* filenames[], int count);
int start_hybrid_simulation();
void cleanup_simulation();
void check_drone_errors();
void kill_all_processes();

void handle_sigint(int sig) {
    (void)sig;
    printf("\n[!] Simulation interrupted by user. Cleaning up...\n");
    simulation_interrupted = 1;
    cleanup_simulation();
    _exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <drone_script1.txt> <drone_script2.txt> ...\n", argv[0]);
        return EXIT_FAILURE;
    }

    signal(SIGINT, handle_sigint);

    printf("=======================================================\n");
    printf("                DRONE SIMULATION SYSTEM                \n");
    printf("=======================================================\n");
    
    if (init_shared_memory() != 0) {
        fprintf(stderr, "Error: Failed to initialize shared memory\n");
        return EXIT_FAILURE;
    }
    
    if (init_semaphores() != 0) {
        fprintf(stderr, "Error: Failed to initialize semaphores\n");
        cleanup_shared_memory();
        return EXIT_FAILURE;
    }
    
    if (loading_files(&argv[1], argc - 1) != 0) {
        fprintf(stderr, "Error: Could not load drone scripts.\n");
        cleanup_semaphores();
        cleanup_shared_memory();
        return EXIT_FAILURE;
    }
    
    SharedMemory* shm = get_shared_memory();
    printf("Enter collision detection radius: ");
    scanf("%lf", &shm->radius_collision);
    
    int result = start_hybrid_simulation();
    
    if (!error_already_handled) {
        check_drone_errors();
    }
    
    if (shm->script_error_detected || shm->emergency_stop_requested) {
        if (!error_already_handled) {
            printf("\n[!] Simulation terminated due to errors.\n");
            printf("[!] Please correct the script errors before running the simulation again.\n");
        }
        cleanup_simulation();
        exit(EXIT_FAILURE); 
    }
    
    if (result != 0) {
        fprintf(stderr, "Error: Simulation failed\n");
        cleanup_simulation();
        exit(EXIT_FAILURE);
    }
    
    if (!shm->script_error_detected && !shm->emergency_stop_requested) {
        printf("\n=== Generating final simulation report ===\n");
        create_report(shm);
        printf("Simulation completed successfully\n");
    } else {
        printf("\n[!] Simulation terminated with errors - no final report generated\n");
    }
    
    cleanup_simulation();
    return EXIT_SUCCESS;
}

void check_drone_errors() {
    SharedMemory* shm = get_shared_memory();
    bool errors_found = false;
    
    printf("\n=== Checking for drone script errors ===\n");
    
    if (shm->script_error_detected) {
        fprintf(stderr, "[ERROR] Script validation error: %s\n", shm->global_error_message);
        errors_found = true;
    }
    
    for (int i = 0; i < shm->num_drones; i++) {
        if (shm->drones[i].error) {
            fprintf(stderr, "[ERROR] Drone %d reported an error: %s\n", 
                    shm->drones[i].id, shm->drones[i].error_message);
            errors_found = true;
        }
    }
    
    if (!errors_found) {
        printf("No script errors detected.\n");
    }
    
    printf("=======================================\n\n");
    
    if (errors_found) {
        printf("[!] Simulation terminated due to script errors.\n");
        printf("[!] Please correct the script errors before running the simulation again.\n");
        
        kill_all_processes();
        
        error_already_handled = true;
        
        exit(EXIT_FAILURE);
    }
}

void kill_all_processes() {
    for (int i = 0; i < num_drone_processes; i++) {
        if (drone_pids[i] > 0) {
            if (kill(drone_pids[i], 0) == 0) {
                printf("Killing drone process %d\n", drone_pids[i]);
                kill(drone_pids[i], SIGKILL);
                waitpid(drone_pids[i], NULL, 0);
            }
            drone_pids[i] = -1;
        }
    }
}

int loading_files(char* filenames[], int count) {
    SharedMemory* shm = get_shared_memory();
    
    sem_wait(sem_mutex);
    
    for (int i = 0; i < count && i < MAX_DRONES; i++) {
        strncpy(shm->drones[i].script_drone, filenames[i], 
                sizeof(shm->drones[i].script_drone) - 1);
        shm->drones[i].script_drone[sizeof(shm->drones[i].script_drone) - 1] = '\0';
        
        int extracted_id = -1;
        sscanf(filenames[i], "drone_%d", &extracted_id);
        if (extracted_id < 0) {
            extracted_id = i + 1;
        }
        
        shm->drones[i].id = extracted_id;
        shm->drones[i].total_steps = count_lines(filenames[i]);
        shm->drones[i].active = true;
        shm->drones[i].completed = false;
        shm->drones[i].current_step = 0;
        shm->drones[i].error = false;
        memset(shm->drones[i].error_message, 0, MAX_ERROR_MSG_LEN);
        
        if (shm->drones[i].total_steps > shm->max_simulation_steps) {
            shm->max_simulation_steps = shm->drones[i].total_steps;
        }
        
        shm->num_drones++;
    }
    
    sem_post(sem_mutex);
    
    return 0;
}

int start_hybrid_simulation() {
    SharedMemory* shm = get_shared_memory();
    
    printf("Starting simulation with %d drones\n", shm->num_drones);
    printf("\n");
    
    if (init_threads(&thread_manager) != 0) {
        fprintf(stderr, "Error: Failed to initialize threads\n");
        return -1;
    }
    
    for (int i = 0; i < shm->num_drones; i++) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("fork failed");
            return -1;
        }
        
        if (pid == 0) {
            // Child process
            drone_process(i, shm->drones[i].script_drone);
            exit(EXIT_SUCCESS);
        } else {
            // Parent process
            drone_pids[num_drone_processes++] = pid;
        }
    }
    
    printf("Starting simulation...\n\n");

    wait_for_threads(&thread_manager);

    if (shm->emergency_stop_requested || shm->script_error_detected) {
        if (!error_already_handled) {
            printf("\n[!] Simulation terminated due to errors.\n");
            printf("[!] Please correct the script errors before running the simulation again.\n");
            kill_all_processes();
            error_already_handled = true;
        }
        return -1;
    }

    printf("Simulation threads completed, cleaning up drone processes\n");
    printf("\n");
    for (int i = 0; i < num_drone_processes; i++) {
        if (drone_pids[i] > 0) {
            int status;
            pid_t result = waitpid(drone_pids[i], &status, WNOHANG);
            if (result == 0) {
                printf("Terminating drone process %d\n", drone_pids[i]);
                kill(drone_pids[i], SIGTERM);
                
            
                result = waitpid(drone_pids[i], &status, WNOHANG);
                if (result == 0) {
                    printf("Force killing drone process %d\n", drone_pids[i]);
                    kill(drone_pids[i], SIGKILL);
                    waitpid(drone_pids[i], &status, 0);
                }
            }
            printf("Drone process %d terminated\n", drone_pids[i]);
            printf("\n");
            drone_pids[i] = -1;
        }
    }

    return 0;
}

void cleanup_simulation() {
    printf("Starting simulation cleanup...\n");
    printf("\n");
    
    kill_all_processes();
    
    cleanup_threads(&thread_manager);
    
    cleanup_semaphores();
    cleanup_shared_memory();
    
    printf("Cleanup completed\n");
}
