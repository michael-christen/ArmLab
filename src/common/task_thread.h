#ifndef TASK_THREAD_H
#define TASK_THREAD_H


// Creates a thread, and allows tasks to be run on that thread
// Created in particular for ensuring all GL rendering operations occur on a single thread


typedef struct task_thread task_thread_t;
task_thread_t * task_thread_create();

// scheduling a task after thread_destroy() happens results in undefined behavior
void task_thread_schedule_blocking(task_thread_t * tt, void (*f)(void * arg), void * arg);

void task_thread_destroy(task_thread_t * tt);


#endif
