#ifndef __cplusplus
#include <stdbool.h>
#include <math.h>
#else
#include <cmath>
#endif

typedef struct jerkType
{
    double timing;
    double Accelerate;
} jerkType;

typedef struct VAJ_set
{
    double velocity;
    double accelerate;
    double jerk;
} VAJ_set;

void VAJ_process(
    double distance, 
    VAJ_set VAJ, 
    jerkType *Jerk_array
);

double velocity_process(
    double dt, 
    int i, 
    const jerkType Jerk_array[8], 
    bool *finished, 
    bool *t4_ed
) __attribute__ ((const));
