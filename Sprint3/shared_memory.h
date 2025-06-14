#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <semaphore.h>
#include <stdbool.h>
#include <pthread.h>

#define MAX_DRONES 100
#define MAX_COLLISIONS 5
#define MAX_ERROR_MSG_LEN 256
#define SHM_NAME "/drone_simulation_shm"
#define SEM_STEP_NAME "/drone_step_sem"
#define SEM_START_STEP_NAME "/drone_start_step_sem"
#define SEM_STEP_COMPLETE_NAME "/drone_step_complete_sem"
#define SEM_BARRIER_NAME "/drone_barrier_sem"
#define SEM_COLLISION_NAME "/drone_collision_sem"
#define SEM_REPORT_NAME "/drone_report_sem"
#define SEM_MUTEX_NAME "/drone_mutex_sem"

typedef struct {
    double x, y, z;
    double time;
} Position;

typedef struct {
    int drone1_id, drone2_id;
    double distance, time;
    double dr_x1, dr_y1, dr_z1;
    double dr_x2, dr_y2, dr_z2;
} Collisions;

typedef struct {
    int id;
    double x, y, z;
    double position_time;
    bool active;
    bool completed;
    int current_step;
    int total_steps;
    char script_drone[256];
    
    bool error;
    char error_message[MAX_ERROR_MSG_LEN];
} SharedDrone;

typedef struct {
    SharedDrone drones[MAX_DRONES];
    int num_drones;
    
    volatile bool simulation_running;
    volatile bool collision_occurred;
    volatile int current_simulation_step;
    volatile int max_simulation_steps;
    
    Collisions collisions[MAX_COLLISIONS];
    int num_collisions;
    double radius_collision;
    
    volatile bool collision_thread_active;
    volatile bool report_thread_active;
    volatile bool monitor_thread_active;
    volatile bool step_completed;
    
    volatile int drones_ready_count;
    volatile int drones_step_completed;
    
    volatile bool new_collision_detected;
    volatile bool position_data_updated;
    volatile bool report_requested;
    volatile bool emergency_stop_requested;
    
    volatile bool collision_event_pending;
    volatile int latest_collision_index;
    volatile bool immediate_report_requested;
    
    volatile int current_time_step; 
    volatile int drones_waiting_for_step;
    volatile int drones_completed_step;
    volatile bool step_in_progress;
    volatile bool all_drones_ready;
    
    volatile int barrier_count;
    volatile int current_barrier_step;
    volatile bool barrier_release;
    
    volatile bool script_error_detected;
    char global_error_message[MAX_ERROR_MSG_LEN];
    
} SharedMemory;

typedef struct {
    pthread_mutex_t collision_mutex;
    pthread_cond_t collision_cond;
    pthread_mutex_t report_mutex;
    pthread_cond_t report_cond;
    pthread_mutex_t monitor_mutex;
    pthread_cond_t monitor_cond;
    pthread_mutex_t data_mutex;
    pthread_cond_t data_cond;
    
    pthread_mutex_t collision_event_mutex;
    pthread_cond_t collision_event_cond;
    
    pthread_mutex_t step_mutex;
    pthread_cond_t step_cond;
} ThreadSync;

int init_shared_memory(void);
int cleanup_shared_memory(void);
SharedMemory* get_shared_memory(void);
int init_semaphores(void);
int cleanup_semaphores(void);
int init_thread_sync(ThreadSync* sync);
int cleanup_thread_sync(ThreadSync* sync);

extern sem_t *sem_step;
extern sem_t *sem_start_step;
extern sem_t *sem_step_complete;
extern sem_t *sem_barrier;
extern sem_t *sem_collision;
extern sem_t *sem_report;
extern sem_t *sem_mutex;

#endif
