
//
//  Controller.c
//  Source written by Justin Yu
//

#include "Controller.h"
#include <math.h>

static void forward(float dist, int speed);
static void backward(float dist, int speed);
static void left(int angle, float radius, int speed);
static void right(int angle, float radius, int speed);

/**
 * Converts from centimeters to BackEMF ticks (robot measurement).
 * @param  cm a distance in centimeters that you want to convert
 * @return    the parameter `cm` in BackEMF ticks
 */
long CMtoBEMF(float cm) {
    return (long)(cm * 1150. / (M_PI * controller.wheel_diameter));
}

/**
 * Converts from BackEMF ticks to centimeters.
 * @param  ticks a distance in BackEMF ticks that you want to convert
 * @return       the parameter `ticks` in centimeters
 */
float BEMFtoCM(long ticks) {
    return (float)(ticks * (M_PI * controller.wheel_diameter) / 1100.);
}

static void forward(float dist, int speed) {
    if(dist < 0.) {
        backward(-dist, speed);
        return;
    }

    // Calculate the # of ticks the robot must move for each wheel
    long ticks = CMtoBEMF(dist);
    long totalLeftTicks = gmpc(controller.motor_left) + ticks;
    long totalRightTicks = gmpc(controller.motor_right) + ticks;

    // Start motors
    mav(controller.motor_left, speed * LEFT_MOTOR_CORRECTION);
    mav(controller.motor_right, speed * RIGHT_MOTOR_CORRECTION);

    // Keep moving until both motors reach their desired # of ticks
    int sum = 0;
    while(gmpc(controller.motor_left) < totalLeftTicks
          && gmpc(controller.motor_right) < totalRightTicks) {
        sum += gyro_x();
        printf("%d\n", gyro_x());
        if (gmpc(controller.motor_left) >= totalLeftTicks || gmpc(controller.motor_right) >= totalRightTicks)
            break;
    }

    off(controller.motor_left);
    off(controller.motor_right);
}

static void backward(float dist, int speed) {
    if(dist < 0.) {
        forward(-dist, speed);
        return;
    }

    // Calculate the # of ticks the robot must move for each wheel
    long ticks = CMtoBEMF(dist);
    long totalLeftTicks = gmpc(controller.motor_left) - ticks;
    long totalRightTicks = gmpc(controller.motor_right) - ticks;

    // Start motors
    mav(controller.motor_left, -speed);
    mav(controller.motor_right, -speed);

    // Keep moving until both motors reach their desired # of ticks
    while(gmpc(controller.motor_left) > totalLeftTicks
          && gmpc(controller.motor_right) > totalRightTicks) {
        if (gmpc(controller.motor_left) <= totalLeftTicks || gmpc(controller.motor_right) <= totalRightTicks)
            break;
    }
    off(controller.motor_left);
    off(controller.motor_right);
}

static void left(int angle, float radius, int speed) {
    // calculate radii
    float left_radius = radius;
    float right_radius = radius + controller.distance_between_wheels;

    if(left_radius < 0)
        return;
    if(right_radius <= 0)
        return;

    // calculate distance in CM
    float right_distance = (right_radius * M_PI) * ((float)(angle) / 180.);

    // calculate relative velocity

    int left_speed = left_radius * (float)speed / right_radius;
    int right_speed = speed;

    long right_distance_ticks = CMtoBEMF(right_distance);

    // clear motor tick counter
    controller.cmpc(controller.motor_left);
    controller.cmpc(controller.motor_right);

    // power motors

    controller.motor(controller.motor_left, left_speed);
    controller.motor(controller.motor_right, right_speed);

    while(abs(controller.gmpc(controller.motor_right)) <= abs(right_distance_ticks)) {
        msleep(50);
    }

    freeze(controller.motor_left);
    freeze(controller.motor_right);
}


static void right(int angle, float radius, int speed) {
    // calculate radii
    float left_radius = radius + controller.distance_between_wheels;
    float right_radius = radius;

    if(left_radius <= 0)
        return;
    if(right_radius < 0)
        return;

    // calculate distance in CM
    float left_distance = (left_radius * M_PI) * ((float)(angle) / 180.);

    // calculate relative velocity

    int left_speed = speed;
    int right_speed = right_radius * (float)speed / left_radius;

    long left_distance_ticks = CMtoBEMF(left_distance);

    // clear motor tick counter
    controller.cmpc(controller.motor_left);
    controller.cmpc(controller.motor_right);

    // power motors

    controller.motor(controller.motor_left, left_speed);
    controller.motor(controller.motor_right, right_speed);

    while(abs(controller.gmpc(controller.motor_left)) <= abs(left_distance_ticks)) {
        msleep(50);
    }

    freeze(controller.motor_left);
    freeze(controller.motor_right);
}

// FIX THIS
static void slow_servo(int port, int position, float time) {
    float increment = .01;
    float curr, start = controller.get_servo_position(port);
    float i = ((position - start) / time) * increment;
    curr = start;
    if (start > position)
    {
        while(curr > position)
        {
            controller.servo(port, curr);
            curr += i;
            msleep((long)(increment * 1000));
        }
    }
    else if (start < position)
    {
        while(curr < position)
        {
            controller.servo(port, curr);
            curr += i;
            msleep((long)(increment * 1000));
        }
    }
    controller.servo(port, position);
}

// Constructors

Controller new_controller(int motor_left, int motor_right, float distance_between_wheels, float wheel_diameter) {
    Controller instance = {

        // Instance Variables (with no setters)
        .distance_between_wheels = distance_between_wheels,
        .wheel_diameter = wheel_diameter,

        // Assign method references

        .motor_left = motor_left,
        .motor_right = motor_right,
        .forward = &forward, .backward = &backward,
        .left = &left, .right = &right,
        .motor = &motor,
        .mav = &mav, .mtp = &mtp, .mrp = &mrp,
        .stop = &ao,
        .motor_off = &off,
        .gmpc = &gmpc,
        .clear_motor_position_counter = &cmpc,
        .gmpc = &gmpc, .cmpc = &cmpc,
        .enable_servo = &enable_servo,
        .disable_servo = &disable_servo,
        .enable_servos = &enable_servos,
        .disable_servos = &disable_servos,
        .get_servo_position = &get_servo_position,
        .servo = &set_servo_position,
        .slow_servo = &slow_servo,
        .digital = &digital,
        .analog = &analog,
        .analog10 = &analog10,
        .analog_et = &analog_et
    };

    controller = instance;

    return instance;
}

Controller new_create_controller() {
    Controller instance = {
        // Assign method references
        .motor = &motor,
        .mav = &mav, .mtp = &mtp, .mrp = &mrp,
        .stop = &ao,
        .motor_off = &off,
        .gmpc = &gmpc,
        .clear_motor_position_counter = &cmpc,
        .gmpc = &gmpc, .cmpc = &cmpc,
        .enable_servo = &enable_servo,
        .disable_servo = &disable_servo,
        .enable_servos = &enable_servos,
        .disable_servos = &disable_servos,
        .get_servo_position = &get_servo_position,
        .servo = &set_servo_position,
        .slow_servo = &slow_servo,
        .digital = &digital,
        .analog = &analog,
        .analog10 = &analog10,
        .analog_et = &analog_et
    };
    controller = instance;
    return instance;
}

int check_yellow() {
    camera_update();
    camera_update();

    int i;
    float avg = 0;
    for(i = 0; i < 30; i++) {
        camera_update();
        avg += get_object_area(0, 0);
        msleep(50);
    }
    avg /= (float)i;

    if(avg > 1500) {
          // yellow
        printf("yellow: %d\n", avg);
        return 1;
    } else {
        printf("yellow not there: %d\n", avg);
        return 0;
    }
}

float *line_follow_step(enum Side line_side, float params[2]) {
    // TODO: make these constants
    int tophat = 0;
    float kp = .6, // proportionality constant .003 //2.4
    ki = 0.1,  // integral constant .1 //.1
    kd = 1; // derivative constant 1 //8.2
    int low, high;
    float offset = 700; // TODO: add calibration
    int tp = 1100; // speed of motors at error = 0
    float integral = params[0]; // running total of errors
    float last_error = params[1];
    float derivative = 0; // current error - last error

    int sensor_value, turn;
    float error;

    sensor_value = analog(tophat);
    error = sensor_value - offset;
    integral = (7. / 8.) * integral + error;
    derivative = error - last_error;
    turn = (int)(kp * error + ki * integral + kd * derivative) / 2;
    mav(controller.motor_right, tp + turn * (line_side == Left ? 1 : -1));
    mav(controller.motor_left, tp - turn * (line_side == Left ? 1 : -1));
    last_error = error;

    float output[] = {integral, last_error};
    return output;
}

void line_follow(int cm, enum Side line_side) {
    /*
    int distance = CMtoBEMF(cm);

    cmpc(controller.motor_left);
    cmpc(controller.motor_right);

    float initial[] = {0, 0};
    float *params = line_follow_step(line_side, initial);
    while(abs(gmpc(controller.motor_left)) < distance || abs(gmpc(controller.motor_right)) < distance) {
		params = line_follow_step(line_side, params);
        msleep(1);
    }

    freeze(controller.motor_left);
    freeze(controller.motor_right);
    */

    // initialize variables

    int distance = CMtoBEMF(cm);
    printf("%d\n", distance);
    int tophat = 0;
    int et = 1, et_threshold = 1700; // greater than 1600 means that the pole is there

    // PID

    float kp = .7, // proportionality constant .003 //2.4
    ki = 0.1,  // integral constant .1 //.1
    kd = 1; // derivative constant 1 //8.2
    int low, high;
    float offset = 700; // average of white and black sensor values (660, 985) = 822.5
    int tp = 1100; // speed of motors at error = 0
    float integral = 0; // running total of errors
    float last_error = 0;
    float derivative = 0; // current error - last error

    int sensor_value, turn;
    float error;
    cmpc(controller.motor_left);
    cmpc(controller.motor_right);
    while(1) {
        sensor_value = analog(tophat);
        error = sensor_value - offset;
        integral = (7. / 8.) * integral + error;
        derivative = error - last_error;
        turn = (int)(kp * error + ki * integral + kd * derivative) / 2;
        mav(controller.motor_right, tp + turn * (line_side == Left ? 1 : -1));
        mav(controller.motor_left, tp - turn * (line_side == Left ? 1 : -1));
        last_error = error;
        msleep(1);
        if(abs(gmpc(controller.motor_left)) > distance || abs(gmpc(controller.motor_right)) > distance) {
            freeze(controller.motor_left);
            freeze(controller.motor_right);
            break;
        }
    }
}

int line_follow_with_camera(enum Side line_side, enum Side et_side) {
    // initialize variables

    int tophat = 0;

    // PID

    float kp = .7, // proportionality constant .003 //2.4
    ki = 0.1,  // integral constant .1 //.1
    kd = 1; // derivative constant 1 //8.2
    int low, high;
    float offset = 700; // average of white and black sensor values (660, 985) = 822.5
    int tp = 1100; // speed of motors at error = 0
    float integral = 0; // running total of errors
    float last_error = 0;
    float derivative = 0; // current error - last error

    int sensor_value, turn;
    float error;

    while(1) {
        // check camera
        sensor_value = analog(tophat);
        error = sensor_value - offset;
        integral = (7. / 8.) * integral + error;
        derivative = error - last_error;
        turn = (int)(kp * error + ki * integral + kd * derivative) / 2;
        mav(controller.motor_right, tp + turn * (line_side == Left ? 1 : -1));
        mav(controller.motor_left, tp - turn * (line_side == Left ? 1 : -1));
        last_error = error;
        msleep(1);

        if(analog(et_side == Left ? ET_LEFT : ET_RIGHT) > ET_THRESHOLD) {
            freeze(controller.motor_left);
            freeze(controller.motor_right);
            break;
        }
    }

    return check_yellow();
}

void line_follow_until_ET(enum Side line_side, enum Side et_side) {

    int tophat = 0;

    float kp = .7, // proportionality constant .003 //2.4
    ki = 0.1,  // integral constant .1 //.1
    kd = 1; // derivative constant 1 //8.2
    int low, high;
    float offset = 700; // average of white and black sensor values (660, 985) = 822.5
    int tp = 1100; // speed of motors at error = 0
    float integral = 0; // running total of errors
    float last_error = 0;
    float derivative = 0; // current error - last error

    int sensor_value, turn;
    float error;
    cmpc(controller.motor_left);
    cmpc(controller.motor_right);
    while(1) {
        sensor_value = analog(tophat);
        error = sensor_value - offset;
        integral = (7. / 8.) * integral + error;
        derivative = error - last_error;
        turn = (int)(kp * error + ki * integral + kd * derivative) / 2;
        mav(controller.motor_right, tp + turn * (line_side == Left ? 1 : -1));
        mav(controller.motor_left, tp - turn * (line_side == Left ? 1 : -1));
        last_error = error;
        msleep(1);
        if(analog(et_side == Left ? ET_LEFT : ET_RIGHT) > ET_THRESHOLD) {
            freeze(controller.motor_left);
            freeze(controller.motor_right);
            break;
        }
    }

    /*
    float initial[] = {0, 0};
    float *params = line_follow_step(line_side, initial);
	while(analog(et_side == Left ? ET_LEFT : ET_RIGHT) < ET_THRESHOLD) {
		params = line_follow_step(line_side, params);
    }

    freeze(controller.motor_left);
    freeze(controller.motor_right);
    */
}

void forward_accel(int cm) {
    // initialize variables
    int distance = CMtoBEMF(abs(cm));
    printf("%d\n", distance);
    int tophat = 0;

    // PID
    float kp = 0.8, // proportionality constant .5
    ki = 0.1,  // integral constant .1 //.1
    kd = 0.4; // derivative constant 1 //8.2
    int tp = 1100; // speed of motors at error = 0
    if(cm < 0) {
       tp *= -1;
    }
    float integral = 0; // running total of errors
    float last_error = 0;
    float derivative = 0; // current error - last error

    float avg_straight = 34.5; // average value of gyro_x going straight
    // if > avg_straight, drifting LEFT
    // if < avg_straight, drifting RIGHT
    int turn = 0;
    int i;
    float error;
    cmpc(controller.motor_left);
    cmpc(controller.motor_right);
    float avg_x;
    while(1) {
        // sensor_value = accel_y();
        //printf("gyro_x: %d, gyro_y: %d\n", gyro_x(), gyro_y());
        avg_x = 0;
        derivative = 0;
        // take avg
        float num_samples = 100.;
        for(i = 0; i < (int)num_samples; i++) {
            int temp = accel_x();
            if(abs(temp - 45) < 30 || abs(temp - 29) < 30) {
            	avg_x += temp;
            } else {
                num_samples -= 1;
            }
            msleep(1);
        }
        avg_x /= num_samples;

        error = avg_x - avg_straight;
        integral = (5. / 8.) * integral + error;
        derivative = error - last_error;
        turn = (int)(kp * error + ki * integral + kd * derivative);

        mav(controller.motor_right, (tp * (cm < 0 ? 0.93 : 0.98) - turn * 4 * (cm < 0 ? -1 : 1)));
        mav(controller.motor_left, (tp + turn * 4 * (cm < 0 ? -1 : 1)));
        last_error = error;

        if(abs(gmpc(controller.motor_left)) > distance || abs(gmpc(controller.motor_right)) > distance) {
            freeze(controller.motor_left);
            freeze(controller.motor_right);
            break;
        }
    }
}

void backward_accel(int cm) {
    forward_accel(-cm);
}
