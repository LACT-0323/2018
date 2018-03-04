#include <kipr/botball.h>
#include "Controller.h"

enum Side {
    LEFT = 0,
    RIGHT = 1
};

void PID(int distance) {
    // initialize variables

    int tophat = 0;
    int et = 1, et_threshold = 1700; // greater than 1600 means that the pole is there

    // PID

    float kp = 0.003, // proportionality constant
    ki = 0.1,  // integral constant
    kd = 1.0; // derivative constant
    int low, high;
    float offset = 0; // average of white and black sensor values (660, 985) = 822.5
    int tp = 1100; // speed of motors at error = 0
    float integral = 0; // running total of errors
    float last_error = 0;
    float derivative = 0; // current error - last error

    int sensor_value, turn;
    float error;

    low = 100;
    high = 3000;
    offset = (float)(low + high) / 2.;
    msleep(1000);
    cmpc(1);
    while(1) {
        sensor_value = analog(tophat);
        error = sensor_value - offset;
        integral = (7. / 8.) * integral + error;
        derivative = error - last_error;
        turn = (int)(kp * error + ki * integral + kd * derivative);
        mav(1, tp + turn);
        mav(0, tp - turn);
        last_error = error;
        msleep(1);
        printf("%d\n",gmpc(1));
        if(gmpc(1)>distance){
            break;
        }
    }
}

int main()
{
	controller = new_controller(0, 1, 8, 2);
    //controller.forward(30, 900);
    controller.backward(30, 1200);
    //controller.left(90, 0, 90);
    // PID(10000);
    return 0;
}
