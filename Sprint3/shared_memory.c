#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "shared_memory.h"

static int shm_fd = -1;
static SharedMemory *shared_mem = NULL;

sem_t *sem_step = NULL;
sem_t *sem_start_step = NULL;
sem_t *sem_step_complete = NULL;
sem_t *sem_barrier = NULL;
sem_t *sem_collision = NULL;
sem_t *sem_report = NULL;
sem_t *sem_mutex = NULL;

int init_shared_memory(void) {
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return -1;
    }
    
    if (ftruncate(shm_fd, sizeof(SharedMemory)) == -1) {
        perror("ftruncate failed");
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return -1;
    }
    
    shared_mem = mmap(NULL, sizeof(SharedMemory), 
                      PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_mem == MAP_FAILED) {
        perror("mmap failed");
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return -1;
    }
    
    memset(shared_mem, 0, sizeof(SharedMemory));
    shared_mem->simulation_running = true;
    shared_mem->collision_occurred = false;
    shared_mem->current_simulation_step = 0;
    shared_mem->max_simulation_steps = 0;
    shared_mem->num_collisions = 0;
    shared_mem->collision_thread_active = true;
    shared_mem->report_thread_active = true;
    shared_mem->monitor_thread_active = true;
    shared_mem->step_completed = false;
    shared_mem->drones_ready_count = 0;
    shared_mem->drones_step_completed = 0;
    shared_mem->new_collision_detected = false;
    shared_mem->position_data_updated = false;
    shared_mem->report_requested = false;
    shared_mem->emergency_stop_requested = false;
    
    shared_mem->collision_event_pending = false;
    shared_mem->latest_collision_index = -1;
    shared_mem->immediate_report_requested = false;
    
    shared_mem->current_time_step = 1;  
    shared_mem->drones_waiting_for_step = 0;
    shared_mem->drones_completed_step = 0;
    shared_mem->step_in_progress = false;
    shared_mem->all_drones_ready = false;
    
    shared_mem->barrier_count = 0;
    shared_mem->current_barrier_step = 0;
    shared_mem->barrier_release = false;
    
    shared_mem->script_error_detected = false;
    memset(shared_mem->global_error_message, 0, MAX_ERROR_MSG_LEN);
    
    for (int i = 0; i < MAX_DRONES; i++) {
        shared_mem->drones[i].error = false;
        memset(shared_mem->drones[i].error_message, 0, MAX_ERROR_MSG_LEN);
    }
    
    return 0;
}

int init_semaphores(void) {
    sem_unlink(SEM_STEP_NAME);
    sem_unlink(SEM_START_STEP_NAME);
    sem_unlink(SEM_STEP_COMPLETE_NAME);
    sem_unlink(SEM_BARRIER_NAME);
    sem_unlink(SEM_COLLISION_NAME);
    sem_unlink(SEM_REPORT_NAME);
    sem_unlink(SEM_MUTEX_NAME);
    
    sem_step = sem_open(SEM_STEP_NAME, O_CREAT, 0666, 0);
    if (sem_step == SEM_FAILED) {
        perror("sem_open step failed");
        return -1;
    }
    
    sem_start_step = sem_open(SEM_START_STEP_NAME, O_CREAT, 0666, 0);
    if (sem_start_step == SEM_FAILED) {
        perror("sem_open start_step failed");
        return -1;
    }
    
    sem_step_complete = sem_open(SEM_STEP_COMPLETE_NAME, O_CREAT, 0666, 0);
    if (sem_step_complete == SEM_FAILED) {
        perror("sem_open step_complete failed");
        return -1;
    }
    
    sem_barrier = sem_open(SEM_BARRIER_NAME, O_CREAT, 0666, 0);
    if (sem_barrier == SEM_FAILED) {
        perror("sem_open barrier failed");
        return -1;
    }
    
    sem_collision = sem_open(SEM_COLLISION_NAME, O_CREAT, 0666, 0);
    if (sem_collision == SEM_FAILED) {
        perror("sem_open collision failed");
        return -1;
    }
    
    sem_report = sem_open(SEM_REPORT_NAME, O_CREAT, 0666, 0);
    if (sem_report == SEM_FAILED) {
        perror("sem_open report failed");
        return -1;
    }
    
    sem_mutex = sem_open(SEM_MUTEX_NAME, O_CREAT, 0666, 1);
    if (sem_mutex == SEM_FAILED) {
        perror("sem_open mutex failed");
        return -1;
    }
    
    return 0;
}

int init_thread_sync(ThreadSync* sync) {
    if (pthread_mutex_init(&sync->collision_mutex, NULL) != 0) {
        perror("Failed to initialize collision mutex");
        return -1;
    }
    if (pthread_cond_init(&sync->collision_cond, NULL) != 0) {
        perror("Failed to initialize collision condition variable");
        pthread_mutex_destroy(&sync->collision_mutex);
        return -1;
    }
    
    if (pthread_mutex_init(&sync->report_mutex, NULL) != 0) {
        perror("Failed to initialize report mutex");
        pthread_mutex_destroy(&sync->collision_mutex);
        pthread_cond_destroy(&sync->collision_cond);
        return -1;
    }
    if (pthread_cond_init(&sync->report_cond, NULL) != 0) {
        perror("Failed to initialize report condition variable");
        pthread_mutex_destroy(&sync->collision_mutex);
        pthread_cond_destroy(&sync->collision_cond);
        pthread_mutex_destroy(&sync->report_mutex);
        return -1;
    }
    
    if (pthread_mutex_init(&sync->monitor_mutex, NULL) != 0) {
        perror("Failed to initialize monitor mutex");
        cleanup_thread_sync(sync);
        return -1;
    }
    if (pthread_cond_init(&sync->monitor_cond, NULL) != 0) {
        perror("Failed to initialize monitor condition variable");
        cleanup_thread_sync(sync);
        return -1;
    }
    
    if (pthread_mutex_init(&sync->data_mutex, NULL) != 0) {
        perror("Failed to initialize data mutex");
        cleanup_thread_sync(sync);
        return -1;
    }
    if (pthread_cond_init(&sync->data_cond, NULL) != 0) {
        perror("Failed to initialize data condition variable");
        cleanup_thread_sync(sync);
        return -1;
    }
    
    if (pthread_mutex_init(&sync->collision_event_mutex, NULL) != 0) {
        perror("Failed to initialize collision event mutex");
        cleanup_thread_sync(sync);
        return -1;
    }
    if (pthread_cond_init(&sync->collision_event_cond, NULL) != 0) {
        perror("Failed to initialize collision event condition variable");
        cleanup_thread_sync(sync);
        return -1;
    }
    
    if (pthread_mutex_init(&sync->step_mutex, NULL) != 0) {
        perror("Failed to initialize step mutex");
        cleanup_thread_sync(sync);
        return -1;
    }
    if (pthread_cond_init(&sync->step_cond, NULL) != 0) {
        perror("Failed to initialize step condition variable");
        cleanup_thread_sync(sync);
        return -1;
    }
    
    return 0;
}

int cleanup_thread_sync(ThreadSync* sync) {
    pthread_mutex_destroy(&sync->collision_mutex);
    pthread_cond_destroy(&sync->collision_cond);
    pthread_mutex_destroy(&sync->report_mutex);
    pthread_cond_destroy(&sync->report_cond);
    pthread_mutex_destroy(&sync->monitor_mutex);
    pthread_cond_destroy(&sync->monitor_cond);
    pthread_mutex_destroy(&sync->data_mutex);
    pthread_cond_destroy(&sync->data_cond);
    pthread_mutex_destroy(&sync->collision_event_mutex);
    pthread_cond_destroy(&sync->collision_event_cond);
    pthread_mutex_destroy(&sync->step_mutex);
    pthread_cond_destroy(&sync->step_cond);
    
    printf("Thread synchronization cleaned up\n");
    return 0;
}

SharedMemory* get_shared_memory(void) {
    return shared_mem;
}

int cleanup_shared_memory(void) {
    int result = 0;
    
    if (shared_mem != NULL) {
        if (munmap(shared_mem, sizeof(SharedMemory)) == -1) {
            perror("munmap failed");
            result = -1;
        }
        shared_mem = NULL;
    }
    
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
    
    if (shm_unlink(SHM_NAME) == -1) {
        perror("shm_unlink failed");
        result = -1;
    }
    
    printf("Shared memory cleaned up\n");
    return result;
}

int cleanup_semaphores(void) {
    int result = 0;
    
    if (sem_step != NULL) {
        sem_close(sem_step);
        sem_unlink(SEM_STEP_NAME);
        sem_step = NULL;
    }
    
    if (sem_start_step != NULL) {
        sem_close(sem_start_step);
        sem_unlink(SEM_START_STEP_NAME);
        sem_start_step = NULL;
    }
    
    if (sem_step_complete != NULL) {
        sem_close(sem_step_complete);
        sem_unlink(SEM_STEP_COMPLETE_NAME);
        sem_step_complete = NULL;
    }
    
    if (sem_barrier != NULL) {
        sem_close(sem_barrier);
        sem_unlink(SEM_BARRIER_NAME);
        sem_barrier = NULL;
    }
    
    if (sem_collision != NULL) {
        sem_close(sem_collision);
        sem_unlink(SEM_COLLISION_NAME);
        sem_collision = NULL;
    }
    
    if (sem_report != NULL) {
        sem_close(sem_report);
        sem_unlink(SEM_REPORT_NAME);
        sem_report = NULL;
    }
    
    if (sem_mutex != NULL) {
        sem_close(sem_mutex);
        sem_unlink(SEM_MUTEX_NAME);
        sem_mutex = NULL;
    }
    
    printf("Semaphores cleaned up\n");
    return result;
}
