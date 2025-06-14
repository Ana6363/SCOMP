#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include "thread_manager.h"
#include "shared_memory.h"
#include "utils.h"
#include "collision.h"
#include "report.h"

void kill_all_drones(void);
void signal_all_threads_to_exit(ThreadManager* tm);

void* step_synchronization_thread(void* arg) {
    ThreadManager* tm = (ThreadManager*)arg;
    SharedMemory* shm = get_shared_memory();
    
    int step = 1;
    
    while (shm->simulation_running) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        
        int sem_result = sem_timedwait(sem_step, &ts);
        
        if (!shm->simulation_running) {
            printf("Step thread: Simulation not running, exiting\n");
            break;
        }
        
        if (sem_result != 0) {
            if (errno == ETIMEDOUT) {
                sem_wait(sem_mutex);
                int active_drones = 0;
                int completed_drones = 0;
                
                for (int i = 0; i < shm->num_drones; i++) {
                    if (shm->drones[i].active) active_drones++;
                    if (shm->drones[i].completed) completed_drones++;
                }
                
                if (completed_drones == shm->num_drones || active_drones == 0) {
                    printf("Step thread: All drones completed or no active drones, ending simulation\n");
                    shm->simulation_running = false;
                    sem_post(sem_mutex);
                    signal_all_threads_to_exit(tm);
                    break;
                }
                sem_post(sem_mutex);
                continue; 
            } else {
                perror("sem_timedwait failed in step thread");
                break;
            }
        }
        
        if (!shm->simulation_running) break;
        
        sem_wait(sem_mutex);
        
        if (shm->script_error_detected) {
            printf("Script error detected: %s\n", shm->global_error_message);
            shm->simulation_running = false;
            shm->emergency_stop_requested = true;
            sem_post(sem_mutex);
            
            signal_all_threads_to_exit(tm);
            kill_all_drones();
            
            printf("All threads signaled to exit due to error\n");
            break;
        }
        
        int active_drones = 0;
        int completed_drones = 0;
        
        for (int i = 0; i < shm->num_drones; i++) {
            if (shm->drones[i].active) active_drones++;
            if (shm->drones[i].completed) completed_drones++;
        }
        
        if (active_drones == 0 || completed_drones == shm->num_drones) {
            if (active_drones == 0) {
                printf("No active drones remaining, ending simulation\n");
                printf("\n");
            } else if (completed_drones == shm->num_drones) {
                printf("All drones completed, ending simulation\n");
                printf("\n");
            }
            shm->simulation_running = false;
            sem_post(sem_mutex);
            
            signal_all_threads_to_exit(tm);
            break;
        }
    
        if (shm->drones_waiting_for_step >= active_drones) {
            shm->current_time_step = step;
            shm->step_in_progress = true;
            
            shm->drones_waiting_for_step = 0;
            shm->drones_completed_step = 0;
            
            shm->current_barrier_step = step;
            shm->barrier_count = 0;
            shm->barrier_release = false;

            sem_post(sem_mutex);

            for (int i = 0; i < active_drones; i++) {
                sem_post(sem_start_step);
            }

            sem_wait(sem_step_complete);
            
            sem_wait(sem_mutex);
            if (shm->script_error_detected) {
                printf("Script error detected after step %d: %s\n", 
                       step, shm->global_error_message);
                shm->simulation_running = false;
                shm->emergency_stop_requested = true;
                sem_post(sem_mutex);
                
                signal_all_threads_to_exit(tm);
                kill_all_drones();
                
                printf("All threads signaled to exit due to error\n");
                break;
            }
            sem_post(sem_mutex);
            
            pthread_mutex_lock(&tm->sync.collision_mutex);
            shm->position_data_updated = true;
            pthread_cond_signal(&tm->sync.collision_cond);
            pthread_mutex_unlock(&tm->sync.collision_mutex);

            pthread_mutex_lock(&tm->sync.collision_mutex);
            while (shm->position_data_updated && shm->simulation_running) {
                pthread_cond_wait(&tm->sync.collision_cond, &tm->sync.collision_mutex);
            }
            pthread_mutex_unlock(&tm->sync.collision_mutex);
            
            sem_wait(sem_mutex);
            if (shm->script_error_detected || shm->emergency_stop_requested) {
                printf("Error detected after collision check: %s\n", 
                       shm->global_error_message);
                shm->simulation_running = false;
                sem_post(sem_mutex);
                
                signal_all_threads_to_exit(tm);
                kill_all_drones();
                
                printf("All threads signaled to exit due to error\n");
                break;
            }
            
            shm->barrier_release = true;
            sem_post(sem_mutex);
            
            for (int i = 0; i < active_drones; i++) {
                sem_post(sem_barrier);
            }
            
            step++;
            shm->current_simulation_step = step;
            shm->step_in_progress = false;

            sem_wait(sem_mutex);
            active_drones = 0;
            completed_drones = 0;
            for (int i = 0; i < shm->num_drones; i++) {
                if (shm->drones[i].active) active_drones++;
                if (shm->drones[i].completed) completed_drones++;
            }

            if (active_drones == 0 || completed_drones == shm->num_drones || shm->script_error_detected) {
                if (completed_drones == shm->num_drones) {
                    printf("All drones completed their scripts\n");
                    printf("\n");
                } else if (active_drones == 0) {
                    printf("All remaining drones terminated\n");
                    printf("\n");
                } else if (shm->script_error_detected) {
                    printf("Script error detected: %s\n", shm->global_error_message);
                }
                shm->simulation_running = false;
                sem_post(sem_mutex);
                
                signal_all_threads_to_exit(tm);
                break;
            }

            sem_post(sem_mutex);
        } else {
            sem_post(sem_mutex);
        }
    }

    printf("Step synchronization thread exiting\n");
    
    for (int i = 0; i < 10; i++) {
        sem_post(sem_start_step);
        sem_post(sem_barrier);
    }

    return NULL;
}

void* collision_detection_thread(void* arg) {
    ThreadManager* tm = (ThreadManager*)arg;
    SharedMemory* shm = get_shared_memory();
    
    while (shm->simulation_running && shm->collision_thread_active) {
        if (shm->script_error_detected || shm->emergency_stop_requested) {
            printf("Collision thread: Error detected, terminating\n");
            break;
        }
        
        pthread_mutex_lock(&tm->sync.collision_mutex);
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        
        while (!shm->position_data_updated && shm->simulation_running && shm->collision_thread_active) {
            int wait_result = pthread_cond_timedwait(&tm->sync.collision_cond, &tm->sync.collision_mutex, &ts);
            
            if (wait_result == ETIMEDOUT || !shm->simulation_running || !shm->collision_thread_active || 
                shm->script_error_detected || shm->emergency_stop_requested) {
                break;
            }
        }
        
        if (!shm->simulation_running || !shm->collision_thread_active || 
            shm->script_error_detected || shm->emergency_stop_requested) {
            pthread_mutex_unlock(&tm->sync.collision_mutex);
            printf("Collision thread: Terminating due to simulation stop or error\n");
            break;
        }
        
        if (!shm->position_data_updated) {
            pthread_mutex_unlock(&tm->sync.collision_mutex);
            continue;
        }

        shm->position_data_updated = false;
        pthread_mutex_unlock(&tm->sync.collision_mutex);
        
        sem_wait(sem_mutex);
        
        if (shm->script_error_detected || shm->emergency_stop_requested) {
            printf("Collision thread: Error detected, terminating\n");
            sem_post(sem_mutex);
            
            pthread_mutex_lock(&tm->sync.collision_mutex);
            pthread_cond_signal(&tm->sync.collision_cond);
            pthread_mutex_unlock(&tm->sync.collision_mutex);
            
            break;
        }
        
        int active_drones = 0;
        for (int i = 0; i < shm->num_drones; i++) {
            if (shm->drones[i].active) {
                printf("Drone %d: (%.2f, %.2f, %.2f) [Step %d]\n", 
                       shm->drones[i].id, shm->drones[i].x, shm->drones[i].y, shm->drones[i].z,
                       shm->drones[i].current_step + 1); 
                active_drones++;
            }
        }
        
        if (active_drones > 1) {
            int previous_collision_count = shm->num_collisions;
            
            detect_collisions(shm);
            
            if (shm->num_collisions > previous_collision_count) {
                shm->collision_event_pending = true;
                shm->latest_collision_index = shm->num_collisions - 1;
                shm->immediate_report_requested = true;
                
                sem_post(sem_mutex);
                
                pthread_mutex_lock(&tm->sync.collision_event_mutex);
                pthread_cond_signal(&tm->sync.collision_event_cond);
                pthread_mutex_unlock(&tm->sync.collision_event_mutex);
                
                pthread_mutex_lock(&tm->sync.report_mutex);
                shm->report_requested = true;
                pthread_cond_signal(&tm->sync.report_cond);
                pthread_mutex_unlock(&tm->sync.report_mutex);
                
                sem_wait(sem_mutex);
                if (check_collision_threshold(shm)) {
                    shm->simulation_running = false;
                    
                    pthread_mutex_lock(&tm->sync.monitor_mutex);
                    pthread_cond_signal(&tm->sync.monitor_cond);
                    pthread_mutex_unlock(&tm->sync.monitor_mutex);
                }
                sem_post(sem_mutex);
            } else {
                sem_post(sem_mutex);
            }
        } else if (active_drones <= 1) {
            int completed_drones = 0;
            for (int i = 0; i < shm->num_drones; i++) {
                if (shm->drones[i].completed) completed_drones++;
            }
            
            if (completed_drones == shm->num_drones) {
                shm->simulation_running = false;
                
                pthread_mutex_lock(&tm->sync.monitor_mutex);
                pthread_cond_signal(&tm->sync.monitor_cond);
                pthread_mutex_unlock(&tm->sync.monitor_mutex);
            }
            
            sem_post(sem_mutex);
        } else {
            sem_post(sem_mutex);
        }
        
        pthread_mutex_lock(&tm->sync.collision_mutex);
        pthread_cond_signal(&tm->sync.collision_cond);
        pthread_mutex_unlock(&tm->sync.collision_mutex);
    }
    
    printf("Collision detection thread exiting\n");
    return NULL;
}

void* report_generation_thread(void* arg) {
    ThreadManager* tm = (ThreadManager*)arg;
    SharedMemory* shm = get_shared_memory();
    
    while (shm->simulation_running && shm->report_thread_active) {
        if (shm->script_error_detected || shm->emergency_stop_requested) {
            printf("Report thread: Error detected, terminating\n");
            break;
        }
        
        pthread_mutex_lock(&tm->sync.collision_event_mutex);
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        
        while (!shm->collision_event_pending && !shm->immediate_report_requested && 
               !shm->report_requested && shm->simulation_running && shm->report_thread_active) {
            int wait_result = pthread_cond_timedwait(&tm->sync.collision_event_cond, 
                                                    &tm->sync.collision_event_mutex, &ts);
            
            if (wait_result == ETIMEDOUT || !shm->simulation_running || !shm->report_thread_active || 
                shm->script_error_detected || shm->emergency_stop_requested) {
                break;
            }
        }
        
        if (!shm->simulation_running || !shm->report_thread_active || 
            shm->script_error_detected || shm->emergency_stop_requested) {
            pthread_mutex_unlock(&tm->sync.collision_event_mutex);
            printf("Report thread: Terminating due to simulation stop or error\n");
            break;
        }
        
        if (shm->collision_event_pending) {
            int collision_index = shm->latest_collision_index;
            shm->collision_event_pending = false;
            shm->immediate_report_requested = false;
            
            pthread_mutex_unlock(&tm->sync.collision_event_mutex);
            
            sem_wait(sem_mutex);
            
            if (shm->script_error_detected || shm->emergency_stop_requested) {
                printf("Report thread: Error detected, skipping collision report\n");
                sem_post(sem_mutex);
                continue;
            }
            
            if (collision_index >= 0 && collision_index < shm->num_collisions) {
                Collisions* collision = &shm->collisions[collision_index];
                
                printf("========================================\n");
                printf("        REAL-TIME COLLISION REPORT       \n");
                printf("========================================\n");
                printf("Collision #%d Details (Step %d):\n", 
                       collision_index + 1, shm->current_time_step);
                printf("Drones Involved: %d and %d\n", 
                       collision->drone1_id, collision->drone2_id);
                printf("Time of Collision: %.2f seconds\n", collision->time);
                printf("Distance Between Drones: %.2f meters\n", collision->distance);
                printf("Drone %d Position: (%.2f, %.2f, %.2f)\n", 
                       collision->drone1_id, collision->dr_x1, collision->dr_y1, collision->dr_z1);
                printf("Drone %d Position: (%.2f, %.2f, %.2f)\n", 
                       collision->drone2_id, collision->dr_x2, collision->dr_y2, collision->dr_z2);
                printf("Total Collisions So Far: %d/%d\n", 
                       shm->num_collisions, MAX_COLLISIONS);
                printf("Collision Radius: %.2f meters\n", shm->radius_collision);
                
                for (int i = 0; i < shm->num_drones; i++) {
                    if (shm->drones[i].id == collision->drone1_id || 
                        shm->drones[i].id == collision->drone2_id) {
                        printf("Drone %d Status: %s\n", 
                               shm->drones[i].id, 
                               shm->drones[i].active ? "ACTIVE" : "TERMINATED");
                    }
                }
                
                printf("========================================\n");
                printf("\n");
            }
            
            sem_post(sem_mutex);
        } else {
            pthread_mutex_unlock(&tm->sync.collision_event_mutex);
        }
        
        pthread_mutex_lock(&tm->sync.report_mutex);
        if (shm->report_requested) {
            shm->report_requested = false;
            pthread_mutex_unlock(&tm->sync.report_mutex);
            
            sem_wait(sem_mutex);
            
            if (shm->script_error_detected || shm->emergency_stop_requested) {
                printf("Report thread: Error detected, skipping status report\n");
                sem_post(sem_mutex);
                continue;
            }
            
            printf("======= STEP %d STATUS REPORT =======\n", shm->current_time_step);
            int active_count = 0, completed_count = 0;
            
            for (int i = 0; i < shm->num_drones; i++) {
                if (shm->drones[i].active) {
                    printf("Drone %d: ACTIVE at (%.2f, %.2f, %.2f) [Step %d]\n", 
                           shm->drones[i].id, shm->drones[i].x, shm->drones[i].y, shm->drones[i].z,
                           shm->drones[i].current_step + 1);
                    active_count++;
                } else if (shm->drones[i].completed) {
                    printf("Drone %d: COMPLETED at (%.2f, %.2f, %.2f) [Step %d]\n", 
                           shm->drones[i].id, shm->drones[i].x, shm->drones[i].y, shm->drones[i].z,
                           shm->drones[i].current_step + 1); 
                    completed_count++;
                } else {
                    printf("Drone %d: TERMINATED at (%.2f, %.2f, %.2f) [Step %d]\n", 
                           shm->drones[i].id, shm->drones[i].x, shm->drones[i].y, shm->drones[i].z,
                           shm->drones[i].current_step + 1);
                }
            }
            
            printf("Summary: %d active, %d completed, %d/%d collisions\n", 
                   active_count, completed_count, shm->num_collisions, MAX_COLLISIONS);
            printf("=======================================\n");
            printf("\n");
            
            sem_post(sem_mutex);
        } else {
            pthread_mutex_unlock(&tm->sync.report_mutex);
        }
    }
    
    printf("Report generation thread exiting\n");
    return NULL;
}

void* system_monitor_thread(void* arg) {
    ThreadManager* tm = (ThreadManager*)arg;
    SharedMemory* shm = get_shared_memory();
    
    int monitor_count = 0;
    
    while (shm->simulation_running && shm->monitor_thread_active) {
        if (shm->script_error_detected || shm->emergency_stop_requested) {
            printf("Monitor thread: Error detected, terminating\n");
            break;
        }
        
        pthread_mutex_lock(&tm->sync.monitor_mutex);
        
        if (!shm->simulation_running || !shm->monitor_thread_active) {
            pthread_mutex_unlock(&tm->sync.monitor_mutex);
            break;
        }
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        
        int wait_result = pthread_cond_timedwait(&tm->sync.monitor_cond, &tm->sync.monitor_mutex, &ts);
        
        if (!shm->simulation_running || !shm->monitor_thread_active) {
            pthread_mutex_unlock(&tm->sync.monitor_mutex);
            break;
        }
        
        pthread_mutex_unlock(&tm->sync.monitor_mutex);
        
        sem_wait(sem_mutex);
        
        if (shm->script_error_detected) {
            printf("Monitor thread: Script error detected: %s\n", shm->global_error_message);
            shm->simulation_running = false;
            shm->emergency_stop_requested = true;
            sem_post(sem_mutex);
            
            signal_all_threads_to_exit(tm);
            kill_all_drones();
            break;
        }
        
        if (shm->emergency_stop_requested) {
            printf("Monitor thread: Emergency stop requested. Terminating simulation.\n");
            shm->simulation_running = false;
            sem_post(sem_mutex);
            
            signal_all_threads_to_exit(tm);
            kill_all_drones();
            break;
        }
        
        int active_drones = 0, completed_drones = 0;
        for (int i = 0; i < shm->num_drones; i++) {
            if (shm->drones[i].active) active_drones++;
            if (shm->drones[i].completed) completed_drones++;
        }
        
        if (wait_result == 0) {
            printf("System Health - Step: %d, Active: %d, Completed: %d, Collisions: %d/%d\n", 
                   shm->current_time_step, active_drones, completed_drones, 
                   shm->num_collisions, MAX_COLLISIONS);
        }
        
        if (completed_drones == shm->num_drones) {
            printf("Monitor thread: All drones completed. Ending simulation.\n");
            printf("\n");
            shm->simulation_running = false;
            sem_post(sem_mutex);
            
            sem_post(sem_step);
            break;
        } else if (active_drones == 0 && completed_drones < shm->num_drones) {
            printf("Monitor thread: No active drones remaining. Ending simulation.\n");
            printf("\n");
            shm->simulation_running = false;
            sem_post(sem_mutex);
            
            sem_post(sem_step);
            break;
        }
        
        sem_post(sem_mutex);
        
        monitor_count++;
        if (shm->simulation_running && monitor_count % 2 == 0) {
            pthread_mutex_lock(&tm->sync.report_mutex);
            shm->report_requested = true;
            pthread_cond_signal(&tm->sync.report_cond);
            pthread_mutex_unlock(&tm->sync.report_mutex);
            
            pthread_mutex_lock(&tm->sync.collision_event_mutex);
            pthread_cond_signal(&tm->sync.collision_event_cond);
            pthread_mutex_unlock(&tm->sync.collision_event_mutex);
        }
    }
    
    printf("System monitor thread exiting\n");
    return NULL;
}

void kill_all_drones(void) {
    extern pid_t drone_pids[];
    extern int num_drone_processes;
    
    printf("Killing all drone processes due to error\n");
    
    for (int i = 0; i < num_drone_processes; i++) {
        if (drone_pids[i] > 0) {
            kill(drone_pids[i], SIGKILL);
            waitpid(drone_pids[i], NULL, 0);
        }
    }
}

void signal_all_threads_to_exit(ThreadManager* tm) {
    SharedMemory* shm = get_shared_memory();
    
    shm->collision_thread_active = false;
    shm->report_thread_active = false;
    shm->monitor_thread_active = false;
    
    pthread_mutex_lock(&tm->sync.monitor_mutex);
    pthread_cond_signal(&tm->sync.monitor_cond);
    pthread_mutex_unlock(&tm->sync.monitor_mutex);
    
    pthread_mutex_lock(&tm->sync.collision_mutex);
    pthread_cond_signal(&tm->sync.collision_cond);
    pthread_mutex_unlock(&tm->sync.collision_mutex);
    
    pthread_mutex_lock(&tm->sync.collision_event_mutex);
    pthread_cond_signal(&tm->sync.collision_event_cond);
    pthread_mutex_unlock(&tm->sync.collision_event_mutex);
    
    pthread_mutex_lock(&tm->sync.report_mutex);
    pthread_cond_signal(&tm->sync.report_cond);
    pthread_mutex_unlock(&tm->sync.report_mutex);
    
    for (int i = 0; i < 5; i++) {
        sem_post(sem_step);
        sem_post(sem_start_step);
        sem_post(sem_step_complete);
        sem_post(sem_barrier);
    }
}

void signal_collision_check(ThreadManager* tm) {
    pthread_mutex_lock(&tm->sync.collision_mutex);
    get_shared_memory()->position_data_updated = true;
    pthread_cond_signal(&tm->sync.collision_cond);
    pthread_mutex_unlock(&tm->sync.collision_mutex);
}

void signal_report_generation(ThreadManager* tm) {
    pthread_mutex_lock(&tm->sync.report_mutex);
    get_shared_memory()->report_requested = true;
    pthread_cond_signal(&tm->sync.report_cond);
    pthread_mutex_unlock(&tm->sync.report_mutex);
}

void signal_position_update(ThreadManager* tm) {
    pthread_mutex_lock(&tm->sync.data_mutex);
    pthread_cond_signal(&tm->sync.data_cond);
    pthread_mutex_unlock(&tm->sync.data_mutex);
}

void signal_emergency_stop(ThreadManager* tm) {
    pthread_mutex_lock(&tm->sync.monitor_mutex);
    get_shared_memory()->emergency_stop_requested = true;
    pthread_cond_signal(&tm->sync.monitor_cond);
    pthread_mutex_unlock(&tm->sync.monitor_mutex);
}

void signal_collision_event(ThreadManager* tm) {
    pthread_mutex_lock(&tm->sync.collision_event_mutex);
    get_shared_memory()->collision_event_pending = true;
    pthread_cond_signal(&tm->sync.collision_event_cond);
    pthread_mutex_unlock(&tm->sync.collision_event_mutex);
}

void signal_monitor_check(ThreadManager* tm) {
    pthread_mutex_lock(&tm->sync.monitor_mutex);
    pthread_cond_signal(&tm->sync.monitor_cond);
    pthread_mutex_unlock(&tm->sync.monitor_mutex);
}

int init_threads(ThreadManager* tm) {
    if (init_thread_sync(&tm->sync) != 0) {
        fprintf(stderr, "Failed to initialize thread synchronization\n");
        return -1;
    }
    
    if (pthread_create(&tm->step_thread, NULL, step_synchronization_thread, tm) != 0) {
        perror("Failed to create step synchronization thread");
        cleanup_thread_sync(&tm->sync);
        return -1;
    }
    
    if (pthread_create(&tm->collision_thread, NULL, collision_detection_thread, tm) != 0) {
        perror("Failed to create collision detection thread");
        pthread_cancel(tm->step_thread);
        cleanup_thread_sync(&tm->sync);
        return -1;
    }
    
    if (pthread_create(&tm->report_thread, NULL, report_generation_thread, tm) != 0) {
        perror("Failed to create report generation thread");
        pthread_cancel(tm->step_thread);
        pthread_cancel(tm->collision_thread);
        cleanup_thread_sync(&tm->sync);
        return -1;
    }
    
    if (pthread_create(&tm->monitor_thread, NULL, system_monitor_thread, tm) != 0) {
        perror("Failed to create system monitor thread");
        pthread_cancel(tm->step_thread);
        pthread_cancel(tm->collision_thread);
        pthread_cancel(tm->report_thread);
        cleanup_thread_sync(&tm->sync);
        return -1;
    }
    
    tm->threads_created = true;
    printf("All threads created successfully\n");
    return 0;
}

int cleanup_threads(ThreadManager* tm) {
    if (!tm->threads_created) return 0;
    
    SharedMemory* shm = get_shared_memory();
    
    printf("Initiating thread cleanup...\n");
    
    signal_all_threads_to_exit(tm);
    
    cleanup_thread_sync(&tm->sync);
    
    printf("Threads cleanup completed\n");
    return 0;
}

int wait_for_threads(ThreadManager* tm) {
    if (!tm->threads_created) return 0;
    
    printf("Waiting for threads to complete...\n");
    
    SharedMemory* shm = get_shared_memory();
    
    printf("Waiting for step synchronization thread...\n");
    pthread_join(tm->step_thread, NULL);
    printf("Step synchronization thread completed\n");
    
    signal_all_threads_to_exit(tm);
    
    printf("Waiting for collision detection thread...\n");
    pthread_join(tm->collision_thread, NULL);
    printf("Collision detection thread completed\n");
    
    printf("Waiting for report generation thread...\n");
    pthread_join(tm->report_thread, NULL);
    printf("Report generation thread completed\n");
    
    printf("Waiting for system monitor thread...\n");
    pthread_join(tm->monitor_thread, NULL);
    printf("System monitor thread completed\n");
    
    printf("All threads completed\n");
    printf("\n");
    return 0;
}
