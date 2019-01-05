#ifndef COMMON_H
#define COMMON_H

#include <sched.h>

static inline int _set_cpu(int cpu_nr, pthread_t thr)
{
    // Dont change
    if (cpu_nr < 0)
        return 0;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_nr, &cpuset);
    return pthread_setaffinity_np(thr, sizeof(cpu_set_t), &cpuset);
}

static inline int set_cpu_current(int cpu_nr)
{
    pthread_t current = pthread_self();

    return _set_cpu(cpu_nr, current);
}

static inline int set_cpu(int cpu_nr, thread *thr)
{
    return _set_cpu(cpu_nr, thr->native_handle());
}

#endif
