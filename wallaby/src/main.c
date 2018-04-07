
#include <kipr/botball.h>
#include "Controller.h"

#define CLAW_SERVO 0
#define CLAW_OPEN 1380
#define CLAW_VERY_OPEN 850
#define CLAW_CLOSED 2047
#define CLAW_MEDIUM 1700
#define ARM_SERVO 1
#define ARM_UP 1400
#define ARM_DOWN 0
#define FRISBEE_SERVO 1
#define FRISBEE_LOW 400
#define FRISBEE_MID 1750
#define FRISBEE_UP 2047
#define ET_LEFT 1
#define ET_RIGHT 2

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

int main()
{
	controller = new_controller(0, 1, 7.6, 2);

	camera_open_black();

    // msleep(2000);
    // determine_color();

    int i = 0;
    for(i = 0; i < 30; i++) {
        camera_update();
        msleep(100);
    }

    controller.servo(CLAW_SERVO, CLAW_MEDIUM);
    controller.servo(ARM_SERVO, ARM_DOWN);
    controller.servo(FRISBEE_SERVO, FRISBEE_LOW);
    controller.enable_servos();

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
    backward_accel(3);

    controller.right(86, 0, 1000);
    forward_accel(19);
    line_follow(14, Left);
    msleep(500);

    close_claw();
    msleep(500);
    backward_accel(20);
    controller.right(56, 0, -1000);
    backward_accel(18);
    raise_frisbee();
    controller.left(56, 0, -1000);
    backward_accel(20);
    forward_accel(4);

    if(position == First) {
        controller.right(90, 0, 1000);
        lower_frisbee();
        line_follow_until_ET(Left, Right);
        forward_accel(12);
        controller.left(15, 0, -400);
        backward_accel(5);
        controller.right(15, 0, 400);
        backward_accel(2);
        controller.right(20, 0, 400);
        backward_accel(9);
        controller.right(30, 0, 400);
        open_claw_wide();
        msleep(1000);
        controller.forward(6, 400);
        msleep(1000);
        backward_accel(10);
    }
    else if(position == Second) {
        controller.left(90, 0, 1000);
        lower_frisbee();
        line_follow_until_ET(Right, Left);
        forward_accel(12);
        controller.right(15, 0, -400);
        backward_accel(5);
        controller.left(15, 0, 400);
        backward_accel(2);
        controller.left(20, 0, 400);
        backward_accel(9);
        controller.left(30, 0, 400);

		open_claw_wide();
        msleep(1000);
        controller.forward(6, 400);
        msleep(1000);
        backward_accel(10);
    }
    else if(position == Third) {
        controller.left(90, 0, 1000);
        lower_frisbee();
        line_follow_until_ET(Right, Left);
        forward_accel(10);
        line_follow_until_ET(Right, Left);

        forward_accel(12);
        controller.right(15, 0, -400);
        backward_accel(5);
        controller.left(15, 0, 400);
        backward_accel(2);
        controller.left(20, 0, 400);
        backward_accel(9);
        controller.left(30, 0, 400);

		open_claw_wide();
        msleep(1000);
        controller.forward(6, 400);
        msleep(1000);
        backward_accel(10);

    }

    controller.servo(CLAW_SERVO, CLAW_VERY_OPEN);

	controller.disable_servos();
    return 0;
}
