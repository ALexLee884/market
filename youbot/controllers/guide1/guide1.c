/*
 * File:          guide1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/keyboard.h>
#include <assert.h>


#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define pi 3.1415926
static WbDeviceTag wheel_motors[3];
static WbDeviceTag gripper_motors[3];
static int time_step = 0;
static void initialize() {
  /* necessary to initialize Webots */
  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();
  
  WbDeviceTag camera = wb_robot_get_device("camera");
  assert(camera);
  wb_camera_enable(camera, time_step);
  if(wb_camera_has_recognition(camera)){
    printf("yes\n");
    //wb_camera_recognition_enable(camera, time_step);
  }
  wb_camera_recognition_enable(camera, time_step);

  int width = wb_camera_get_width(camera);
  printf("width: %d\n", width);
  
  
  gripper_motors[0] = wb_robot_get_device("lift motor");
  gripper_motors[1] = wb_robot_get_device("left finger motor");
  gripper_motors[2] = wb_robot_get_device("right finger motor");
  wheel_motors[0] = wb_robot_get_device("left wheel");
  wheel_motors[1] = wb_robot_get_device("right wheel");
  // Specify velocity control mode
  wb_motor_set_position(wheel_motors[0], INFINITY);
  wb_motor_set_position(wheel_motors[1], INFINITY);
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
}

static void stepp() {
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    stepp();
  } while (start_time + sec > wb_robot_get_time());
}

void step(double seconds) {
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms) {
    wb_robot_step(time_step);
    elapsed_time += time_step;
  }
}

void lift(double position) {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], position);
}


void moveFingers(double position) {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], position);
  wb_motor_set_position(gripper_motors[2], position);
}

void moveForwards(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], speed);
}


void turn(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], -speed);
}


void stop(double seconds) {
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
  step(seconds);
}

int main() {
  
  initialize(); 
  passive_wait(2.0);
  int pc = 0;
  wb_keyboard_enable(time_step);
  
  while (true) {
    stepp();
    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          printf("Go forwards\n");
          stop(0.2);
          moveForwards(1.25);
          break;
        case WB_KEYBOARD_DOWN:
          printf("Go backwards\n");
          stop(0.2);
          moveForwards(-1.25);
          break;
        case WB_KEYBOARD_LEFT:
          printf("Turn left\n");
          stop(0.2);
          turn(-2.0);
          break;
        case WB_KEYBOARD_RIGHT:
          printf("Turn right\n");
          stop(0.2);
          turn(2.0);
          break;
        case WB_KEYBOARD_PAGEUP:
          lift(0.5);
          break;
        case WB_KEYBOARD_PAGEDOWN:
          lift(-0.05);
          break;
        case ' ':
          printf("Stop\n");
          stop(0.5);
          break;
        case '+':
        case 388:
        case 65585:
          printf("Grip\n");
          moveFingers(0.01);
          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          moveFingers(0.06);
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
    
  }
  
  wb_robot_cleanup();
  

  return 0;
}