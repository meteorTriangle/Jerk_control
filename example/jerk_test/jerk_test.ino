#include "Jerk_control.h"

jerkType jerkArray[8];
VAJ_set VAJ_op = {75, 20, 50};

void setup()
{
	Serial.begin(9600);
    delay(2000);
    VAJ_op = VAJ_optimize(100, VAJ_op);
    int j = 0;
    bool finished = false;
    bool t4;
    Serial.println("velocity:" + String(VAJ_op.velocity,4));
    Serial.print("accelerate:");
    Serial.println(String(VAJ_op.accelerate,4));
    Serial.print("jerk:");
    Serial.println(String(VAJ_op.jerk,4));
    VAJ_process(100, VAJ_op, (jerkArray));
    
    for(int i = 0;i < 8; i++){
      Serial.println(i);
      Serial.print("timing:" + String((jerkArray)[i].timing, 8) + "     ");
      Serial.print("accelerate:" + String((jerkArray)[i].Accelerate, 8) + "     ");
    }

    while(!finished){
        Serial.println(String(velocity_process(0.004, j, jerkArray, &finished, &t4), 6));
        j++;
        delay(4);
    }

}

void loop()
{


  
}
