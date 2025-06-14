#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include <pthread.h>
#include "shared_memory.h"

typedef struct {
    pthread_t step_thread;
    pthread_t collision_thread;
    pthread_t report_thread;
    pthread_t monitor_thread;
    pthread_t main_simulation_thread;
    bool threads_created;
    ThreadSync sync;
} ThreadManager;

void* step_synchronization_thread(void* arg);
void* collision_detection_thread(void* arg);
void* report_generation_thread(void* arg);
void* system_monitor_thread(void* arg);
void* main_simulation_thread(void* arg);

int init_threads(ThreadManager* tm);
int cleanup_threads(ThreadManager* tm);
int wait_for_threads(ThreadManager* tm);

void signal_collision_check(ThreadManager* tm);
void signal_report_generation(ThreadManager* tm);
void signal_position_update(ThreadManager* tm);
void signal_emergency_stop(ThreadManager* tm);
void signal_collision_event(ThreadManager* tm);

#endif
