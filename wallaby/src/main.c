
#include <kipr/botball.h>
#include "Controller.h"

#define CLAW_SERVO 0
#define CLAW_OPEN 1300
#define CLAW_VERY_OPEN 850
#define CLAW_CLOSED 2047
#define CLAW_MEDIUM 1700
#define ARM_SERVO 1
#define ARM_UP 1400
#define ARM_DOWN 0

void open_claw() {
    controller.servo(CLAW_SERVO, CLAW_OPEN);
    msleep(100);
}

void close_claw() {
    controller.servo(CLAW_SERVO, CLAW_CLOSED);
    msleep(100);
}
/*
// controller.left (50,0, 1000);
    // controller.forward (15, 1000);
    // controller.forward(10, 1200);
    // controller.right(90, 0, 1200);
    line_follow(59);
    open_claw();
    controller.right(104, 0, 1000);
    controller.forward(20, 1000);
    close_claw;
    */
int main()
{
	controller = new_controller(0, 1, 7.6, 2);
    backward_gyro(30);
    forward_gyro(30);
    /*
    controller.servo(CLAW_SERVO, CLAW_MEDIUM);
    controller.servo(ARM_SERVO, ARM_DOWN);
    controller.enable_servos();
    forward_gyro(10);
    line_follow(44);
    open_claw();
    controller.left(109, 0, 1000);
    controller.forward(19, 1000);
    line_follow(30);
    close_claw();
    controller.backward(50, 1000);
    controller.servo(CLAW_SERVO, CLAW_VERY_OPEN);
   */
    return 0;
}
