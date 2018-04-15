
#include <kipr/botball.h>
#include "Controller.h"

#define CLAW_SERVO 0
#define CLAW_OPEN 1380
#define CLAW_VERY_OPEN 850
#define CLAW_CLOSED 2047
#define CLAW_MEDIUM 1700 //the quick brown fox jumped over the lazy dog
#define ARM_SERVO 1
#define ARM_UP 1400
#define ARM_DOWN 0
#define FRISBEE_SERVO 1
#define FRISBEE_LOW 400
#define FRISBEE_MID 1389
#define FRISBEE_UP 1800
#define ET_LEFT 1
#define ET_RIGHT 2
#define TOPHAT 0
#define FRISBEE_FLAT 1350

void open_claw() {
    controller.servo(CLAW_SERVO, CLAW_OPEN);
    msleep(100);
}

void open_claw_wide() {
    controller.servo(CLAW_SERVO, CLAW_VERY_OPEN);
    msleep(100);
}

void close_claw() {
    controller.servo(CLAW_SERVO, CLAW_CLOSED);
    msleep(250);
}

void get_multiplier() {
    controller.servo(FRISBEE_SERVO, FRISBEE_MID);
    msleep(200);
    controller.backward(10, 400);
    controller.slow_servo(FRISBEE_SERVO, FRISBEE_UP, 1.0);
}

void raise_frisbee() {
	controller.servo(FRISBEE_SERVO, FRISBEE_MID);
    msleep(250);
}

void lower_frisbee() {
	controller.servo(FRISBEE_SERVO, FRISBEE_LOW);
    msleep(250);
}

enum Color {
	Yellow,
    Red,
    Green
};

enum Position {
    First,
    Second,
    Third
};

void lower_frisbee_slow() {
	controller.slow_servo(FRISBEE_SERVO, 800, 3.0);
}

void knock_cube() {
    controller.servo(FRISBEE_SERVO, 1200);
    msleep(150);
    thread tid = thread_create(lower_frisbee_slow);
    thread_start(tid);
    backward_accel(20);
    thread_wait(tid);
    thread_destroy(tid);
    raise_frisbee();
    forward_accel(10);
}

int main()
{
	controller = new_controller(0, 1, 7.6, 2);

    controller.servo(CLAW_SERVO, CLAW_VERY_OPEN);
    controller.servo(ARM_SERVO, ARM_DOWN);
    controller.servo(FRISBEE_SERVO, FRISBEE_LOW);
    controller.enable_servos();

	camera_open_black();

    // msleep(2000);
    // determine_color();

    int i = 0;
    for(i = 0; i < 30; i++) {
        camera_update();
        msleep(100);
    }

    controller.servo(CLAW_SERVO, CLAW_MEDIUM);
    msleep(250);

    forward_accel(10);
    int first_position = line_follow_with_camera(Right, Left);
    printf("In position 1? %s\n", first_position ? "Yes" : "No");
    forward_accel(10);
    int second_position = line_follow_with_camera(Right, Left);
    printf("In position 2? %s\n", second_position ? "Yes" : "No");

    enum Position position = (first_position + second_position) == 0 ? Third : (first_position == 1 ? First : Second);
    printf("In position 3? %s\n", position == Third ? "Yes" : "No");

    open_claw();
    camera_close();
    msleep(100);
    backward_accel(1);
    msleep(100);

    controller.right(86, 0, 1000);
    forward_accel(19);
    line_follow(14, Left);
    msleep(500);

    //enum Position position = Second;
    close_claw();
    msleep(500);
    backward_accel(20);
    controller.right(54, 0, -1000);
    backward_accel(18);
    raise_frisbee();
    controller.left(58, 0, -1000);
    backward_accel(20);
    forward_accel(4);

    if(position == First) {
        controller.right(90, 0, 1000);
        lower_frisbee();
        line_follow_until_ET(Left, Right);
        forward_accel(12);
        controller.left(30, 0, -400);
        backward_accel(5);
        controller.left(30, 0, -400);
        backward_accel(2);
        controller.left(20, 0, -400);
    }
    else if(position == Second) {
        controller.left(90, 0, 1000);
        lower_frisbee();
        line_follow_until_ET(Right, Left);
        forward_accel(12);
        controller.right(30, 0, -400);
        backward_accel(3);
        controller.right(30, 0, -400);
        backward_accel(1);
        controller.right(20, 0, -400);
    }
    else if(position == Third) {
        controller.left(90, 0, 1000);
        lower_frisbee();
        line_follow_until_ET(Right, Left);
        forward_accel(10);
        line_follow_until_ET(Right, Left);

        forward_accel(12);
        controller.right(30, 0, -400);
        backward_accel(3);
        controller.right(30, 0, -400);
        backward_accel(1);
        controller.right(20, 0, -400);
    }

    open_claw_wide();
    msleep(1000);
    controller.forward(6, 400);
    msleep(1000);
    backward_accel(10);

    close_claw();
    mav(controller.motor_left, -930);
    mav(controller.motor_right, -1000);
    while(analog(TOPHAT) < 1000) {
       msleep(100);
    }

    if(position == First || position == Second) {
        controller.left(92, 0, 1000);
        line_follow(position == First ? 10 : 45, Right);
        controller.left(92, 0, 1000);
        open_claw_wide();
        forward_accel(20);
        raise_frisbee();
        controller.right(40, 0, -1000);
        close_claw();
        forward_accel(4);
        controller.right(50, 0, -1000);
        backward_accel(20);
        forward_until_ET(1100, Right);
        msleep(500);
        backward_accel(4);
        controller.left(91, 0, 1000);
        get_multiplier();
		forward_accel(15);
        controller.left(95, 0, -1000);
        controller.backward(15, 500);
        forward_accel(10);
        controller.left(90, 0, 1000);

        mav(controller.motor_left, 900);
    	mav(controller.motor_right, 1000);
        while(analog(TOPHAT) < 1000) {
           msleep(1);
        }

        controller.right(93, 0, 1000);
        line_follow(position == First ? 20 : 35, Right);

        controller.right(90, 0, 1000);
        //controller.servo(FRISBEE_SERVO, FRISBEE_FLAT, 1);
        backward_accel(25);
        //raise_frisbee();
        //msleep(500);
        lower_frisbee();
    }
    else if(position == Third) {
        controller.right(92, 0, 1000);
        line_follow(15, Left);
        controller.right(92, 0, 1000);
        forward_accel(15);
        raise_frisbee();

        controller.left(40, 0, -1000);
        close_claw();
        forward_accel(4);
        controller.left(50, 0, -1000);
        backward_accel(20);
        forward_until_ET(1100, Left);
        msleep(500);
        backward_accel(4);
        controller.right(91, 0, 1000);
        get_multiplier();
		forward_accel(15);
        controller.right(95, 0, -1000);
        controller.backward(15, 500);
        forward_accel(10);
        controller.right(90, 0, 1000);

        mav(controller.motor_left, 900);
    	mav(controller.motor_right, 1000);
        while(analog(TOPHAT) < 1000) {
           msleep(1);
        }
        backward_accel(5);

        controller.left(94, 0, 1000);
        line_follow(23, Left);

        controller.left(96, 0, 1000);
        //controller.servo(FRISBEE_SERVO, FRISBEE_FLAT, 1);
        backward_accel(25);
        //raise_frisbee();
        //msleep(500);
        controller.servo(FRISBEE_SERVO, 450);
        msleep(500);
    }

    controller.servo(CLAW_SERVO, CLAW_VERY_OPEN);

	controller.disable_servos();
    return 0;
}
