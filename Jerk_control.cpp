#include "Jerk_control.h"

double quadratic_equation(double a, double b, double c){//ax^2+bx+c
    double root_1 = (-b + sqrt(pow(b, 2) - 4*a*c))/ 2*a;
    double root_2 = (-b - sqrt(pow(b, 2) - 4*a*c))/ 2*a;
    double root_x = root_1 > root_2 ? root_1 : root_2;
    return(root_x);
}

VAJ_set VAJ_optimize(double distance, VAJ_set VAJ){
    VAJ_set VAJ_1 = VAJ;
    if(1){ //new Accelerate     1
        double newAccelerate = sqrt(VAJ.velocity * VAJ.jerk);
        double acc_time = VAJ.velocity / VAJ.accelerate;
        double jerk_time = VAJ.accelerate / VAJ.jerk;
        VAJ_1.accelerate = acc_time <= jerk_time ? newAccelerate : VAJ.accelerate;
    }
    VAJ_set VAJ_2 = VAJ_1;
        //new Velocity              2
        double velocity_time = distance / VAJ_1.velocity;
        double new_accelerate_time = (VAJ_1.velocity / VAJ_1.accelerate) + (VAJ_1.accelerate / VAJ_1.jerk);
        bool new_Velocity = new_accelerate_time > velocity_time;
        double a = VAJ_1.jerk;
        double b = pow(VAJ_1.accelerate, 2);
        double c = -(distance*VAJ_1.accelerate*VAJ_1.jerk);
        double new_accelerate_1 = quadratic_equation(a, b, c);
        double new_accelerate_2 = pow((pow(distance, 2)*VAJ_1.jerk)/4, 1/3);
        double new_accelerate = new_accelerate_2 > VAJ_1.accelerate ? new_accelerate_1 : new_accelerate_2;
        VAJ_2.velocity = new_Velocity ? new_accelerate : VAJ_1.velocity;
    VAJ_set VAJ_3 = VAJ_2;
        double newAccelerate = sqrt(VAJ.velocity * VAJ.jerk);
        double acc_time = VAJ_2.velocity / VAJ_2.accelerate;
        double jerk_time = VAJ_2.accelerate / VAJ_2.jerk;
        VAJ_3.accelerate = acc_time <= jerk_time ? newAccelerate : VAJ_2.accelerate;
    return VAJ_3;


}

void VAJ_process(double distance, VAJ_set VAJ, jerkType *Jerk_array[8]){
    VAJ_set VAJ_new =  VAJ_optimize(distance, VAJ);
    double t1 = VAJ_new.accelerate / VAJ_new.jerk;
    double t2 = VAJ_new.velocity / VAJ_new.accelerate;
    double t4 = distance / VAJ_new.velocity;
    *Jerk_array[0] = {0, 0};
    *Jerk_array[1] = {t1, VAJ_new.accelerate};
    *Jerk_array[2] = {t2, VAJ_new.accelerate};
    *Jerk_array[3] = {t1 + t2, 0};
    *Jerk_array[4] = {t4, 0};
    *Jerk_array[5] = {t4 + t1, -VAJ_new.accelerate};
    *Jerk_array[6] = {t4 + t2, -VAJ_new.accelerate};
    *Jerk_array[7] = {t4 + t1 + t2, 0};
}

double velocity_process(double dt, int i, jerkType Jerk_array[8], bool *finished, bool *t4_ed){
    //timing threshold
        bool timing_searched = false;
        double timing = int(i) * dt;
        int timing_sequence = 0;
        while(!timing_searched){
            timing_searched = timing >= Jerk_array[i].timing;
            timing_sequence++;
        }
        timing_sequence--;
    //finished and t4_ed
        *finished = timing_sequence == 7;
        *t4_ed = timing_sequence >= 4;
    //velocity process
        double velovity_addAll = 0;
        for(int j = 0; j < timing_sequence - 1; j++){
            velovity_addAll = (Jerk_array[i+1].Accelerate + Jerk_array[i].Accelerate) * (Jerk_array[i+1].timing - Jerk_array[i].timing) /2;
        }
        double timing_threshold = (timing - Jerk_array[i].timing) / (Jerk_array[i+1].timing - Jerk_array[i].timing);
        double accelerate_now = (Jerk_array[i+1].Accelerate - Jerk_array[i].Accelerate)*timing_threshold;
        double velocity_end = velovity_addAll + ((accelerate_now + Jerk_array[i+1].Accelerate) * (timing - Jerk_array[i].timing) / 2);\
        return velocity_end;
    
}
