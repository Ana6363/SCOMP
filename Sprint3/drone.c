#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "drone.h"
#include "shared_memory.h"
#include <time.h>
#include <errno.h>

volatile sig_atomic_t drone_down = 0;
static int this_drone_index;
static SharedMemory* shm = NULL;

void handle_sigusr1(int sig) {
    (void)sig;
    drone_down = 1;
}

void drone_process(int drone_index, const char* script_file) {
    this_drone_index = drone_index;
    signal(SIGUSR1, handle_sigusr1);
    signal(SIGTERM, handle_sigusr1);
    signal(SIGINT, handle_sigusr1);
    
    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("Drone: shm_open failed");
        exit(EXIT_FAILURE);
    }
    
    shm = mmap(NULL, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) {
        perror("Drone: mmap failed");
        close(shm_fd);
        exit(EXIT_FAILURE);
    }
    
    sem_t *sem_mutex_local = sem_open(SEM_MUTEX_NAME, 0);
    sem_t *sem_start_step_local = sem_open(SEM_START_STEP_NAME, 0);
    sem_t *sem_step_complete_local = sem_open(SEM_STEP_COMPLETE_NAME, 0);
    sem_t *sem_barrier_local = sem_open(SEM_BARRIER_NAME, 0);
    sem_t *sem_step_local = sem_open(SEM_STEP_NAME, 0);
    
    if (sem_mutex_local == SEM_FAILED || sem_start_step_local == SEM_FAILED || 
        sem_step_complete_local == SEM_FAILED || sem_barrier_local == SEM_FAILED ||
        sem_step_local == SEM_FAILED) {
        perror("Drone: sem_open failed");
        exit(EXIT_FAILURE);
    }
    
    FILE* file = fopen(script_file, "r");
    if (!file) {
        perror("Error opening drone script file");
        exit(EXIT_FAILURE);
    }
    
    printf("Drone %d Process started, reading script: %s\n", shm->drones[drone_index].id, script_file);
    printf("\n");
    
    int total_lines = 0;
    char line[256];
    while (fgets(line, sizeof(line), file) != NULL) {
        total_lines++;
    }
    
    if (total_lines == 0) {
        fprintf(stderr, "\n[ERROR] No valid steps found in script file %s\n", script_file);
        
        sem_wait(sem_mutex_local);
        shm->drones[drone_index].active = false;
        shm->drones[drone_index].completed = false;
        shm->drones[drone_index].error = true;
        strncpy(shm->drones[drone_index].error_message, 
                "Script error: No valid steps found", 
                sizeof(shm->drones[drone_index].error_message) - 1);
        
        shm->script_error_detected = true;
        snprintf(shm->global_error_message, 
                MAX_ERROR_MSG_LEN - 1,
                "Drone %d script has no valid steps. Check script %s",
                shm->drones[drone_index].id, script_file);
        
        shm->simulation_running = false;
        shm->emergency_stop_requested = true;
        
        sem_post(sem_mutex_local);
        
        sem_post(sem_step_local);
        
        fclose(file);
        sem_close(sem_mutex_local);
        sem_close(sem_start_step_local);
        sem_close(sem_step_complete_local);
        sem_close(sem_barrier_local);
        sem_close(sem_step_local);
        munmap(shm, sizeof(SharedMemory));
        close(shm_fd);
        
        _exit(EXIT_FAILURE);
    }
    
    rewind(file);
        
    double last_step_time = -1.0;
    int step = 0;
    double t, x, y, z;
    int line_number = 0;
    
    sem_wait(sem_mutex_local);
    shm->drones[drone_index].x = 0.0;
    shm->drones[drone_index].y = 0.0;
    shm->drones[drone_index].z = 0.0;
    shm->drones[drone_index].position_time = 0.0;
    shm->drones[drone_index].current_step = 0;
    sem_post(sem_mutex_local);
    
    while (step < total_lines && !drone_down && shm->simulation_running) {
        sem_wait(sem_mutex_local);
        if (shm->script_error_detected || shm->emergency_stop_requested) {
            printf("Drone %d: Terminating due to error detected in another drone\n", 
                   shm->drones[drone_index].id);
            shm->drones[drone_index].active = false;
            sem_post(sem_mutex_local);
            break;
        }
        sem_post(sem_mutex_local);
        
        if (fgets(line, sizeof(line), file) == NULL) {
            break; 
        }
        
        line_number++;
        
        if (sscanf(line, "%lf %lf %lf %lf", &t, &x, &y, &z) != 4) {
            fprintf(stderr, "\n[WARNING] Invalid line format in file %s, line %d: %s", 
                    script_file, line_number, line);
            continue; 
        }
        
        if (last_step_time >= 0 && t <= last_step_time) {
            fprintf(stderr, "\n[ERROR] Script error in file %s, line %d: Step time %.2f is not greater than previous step time %.2f\n", 
                    script_file, line_number, t, last_step_time);

            
            sem_wait(sem_mutex_local);
            
            shm->drones[drone_index].active = false;
            shm->drones[drone_index].completed = false;
            
            shm->drones[drone_index].error = true;
            snprintf(shm->drones[drone_index].error_message, 
                    MAX_ERROR_MSG_LEN - 1,
                    "Script error in line %d: Step time %.2f is not greater than previous step time %.2f",
                    line_number, t, last_step_time);
            
            shm->script_error_detected = true;
            snprintf(shm->global_error_message, 
                    MAX_ERROR_MSG_LEN - 1,
                    "Drone %d script has out-of-order steps. Check script %s, line %d",
                    shm->drones[drone_index].id, script_file, line_number);
            
            shm->simulation_running = false;
            shm->emergency_stop_requested = true;
            
            sem_post(sem_mutex_local);
            
            sem_post(sem_step_local);
            
            fclose(file);
            sem_close(sem_mutex_local);
            sem_close(sem_start_step_local);
            sem_close(sem_step_complete_local);
            sem_close(sem_barrier_local);
            sem_close(sem_step_local);
            munmap(shm, sizeof(SharedMemory));
            close(shm_fd);
            
            _exit(EXIT_FAILURE);
        }
        
        int display_step = step + 1;
        
        sem_wait(sem_mutex_local);
        shm->drones_waiting_for_step++;
           
        sem_post(sem_step_local);
        sem_post(sem_mutex_local);
                
        sem_wait(sem_start_step_local);
        
        sem_wait(sem_mutex_local);
        bool should_continue = shm->simulation_running && !drone_down && shm->drones[drone_index].active;
        
        if (shm->script_error_detected || shm->emergency_stop_requested) {
            should_continue = false;
            printf("Drone %d: Terminating due to error detected\n", shm->drones[drone_index].id);
        }
        sem_post(sem_mutex_local);
        
        if (!should_continue) {
            printf("Drone %d Stopping execution (down=%d, sim_running=%d, active=%d)\n", 
                   shm->drones[drone_index].id, drone_down, shm->simulation_running, shm->drones[drone_index].active);
            break;
        }
        
        sem_wait(sem_mutex_local);
        
        if (shm->drones[drone_index].active) {
            shm->drones[drone_index].x = x;
            shm->drones[drone_index].y = y;
            shm->drones[drone_index].z = z;
            shm->drones[drone_index].position_time = t;
            shm->drones[drone_index].current_step = step;
            
            printf("Drone %d Step %d: Position (%.2f, %.2f, %.2f) at time %.2f\n", 
                   shm->drones[drone_index].id, display_step, x, y, z, t);
            printf("\n");
            
            shm->drones_completed_step++;
            
            int active_drones = 0;
            for (int i = 0; i < shm->num_drones; i++) {
                if (shm->drones[i].active) active_drones++;
            }
            
            if (shm->drones_completed_step >= active_drones) {
                sem_post(sem_step_complete_local);
            }
            
            sem_post(sem_mutex_local);
            
            
            sem_wait(sem_mutex_local);
            shm->barrier_count++;
            sem_post(sem_mutex_local);
            
            sem_wait(sem_barrier_local);
            
            sem_wait(sem_mutex_local);
            bool continue_after_barrier = shm->simulation_running && !drone_down && 
                                         shm->drones[drone_index].active && 
                                         !shm->script_error_detected && 
                                         !shm->emergency_stop_requested;
            sem_post(sem_mutex_local);
            
            if (!continue_after_barrier) {
                break;
            }
            
        } else {
            printf("Drone %d Deactivated - stopping execution\n", shm->drones[drone_index].id);
            printf("\n");
            sem_post(sem_mutex_local);
            break;
        }
        
        last_step_time = t;
        
        step++;
    }
    
    sem_wait(sem_mutex_local);
    if (!shm->drones[drone_index].completed && !shm->drones[drone_index].error && 
        !shm->script_error_detected && !shm->emergency_stop_requested) {
        shm->drones[drone_index].completed = true;
        printf("Drone %d Completed script execution successfully\n", shm->drones[drone_index].id);
        printf("\n");
    } else if (shm->script_error_detected || shm->emergency_stop_requested) {
        printf("Drone %d Process terminating due to errors\n", shm->drones[drone_index].id);
    }
    
    shm->drones[drone_index].active = false;
    sem_post(sem_mutex_local);
    
    printf("Drone %d Process exiting\n", shm->drones[drone_index].id);
    printf("\n");
    
    fclose(file);
    sem_close(sem_mutex_local);
    sem_close(sem_start_step_local);
    sem_close(sem_step_complete_local);
    sem_close(sem_barrier_local);
    sem_close(sem_step_local);
    munmap(shm, sizeof(SharedMemory));
    close(shm_fd);
    
    exit(EXIT_SUCCESS);
}
