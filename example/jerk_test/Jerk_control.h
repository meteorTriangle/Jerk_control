#include <math.h>

typedef struct 
{
    double timing;
    double Accelerate;
} jerkType;

typedef struct 
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
    jerkType Jerk_array[8], 
    bool *finished, 
    bool *t4_ed
);

VAJ_set VAJ_optimize(
    double distance, 
    VAJ_set VAJ
);

double quadratic_equation(
    double a, 
    double b, 
    double c
);
