/*
 * File:          guide.c
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
#include <webots/compass.h>
#include <webots/camera_recognition_object.h>
#include <assert.h>


#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define linetime 19.8
#define turntime1 1.727
#define turntime2 1.729
#define pi 1.5707963
int shelf = 1;
int targetshelf = 0;
int shelf_floor = 0;
double start_time, end_time;
double taking_time = 0.0;  //鏀剧殑鏃堕棿
double putting_time = 0.0; //鍙栫殑鏃堕棿
static WbDeviceTag wheel_motors[3];
static WbDeviceTag gripper_motors[3];
static WbDeviceTag compass;
static WbDeviceTag camera;
static WbDeviceTag ds_top;
static WbDeviceTag so6;
static WbDeviceTag so7;
static WbDeviceTag so8;
static WbDeviceTag ds_front1;
static WbDeviceTag ds_front2;
static int time_step = 0;
const double *direction;
const WbCameraRecognitionObject *object;
double value_top,value6,value7,value8, value_front1,value_front2;
double lift_degree = 0.0;
double finger_degree = 0.0;
double high = 0.0;
int i = 0;
double water = 1.0;
double greencan = 0.9;
double redcan = 0.8;
double beer = 0.7;
double jerry = 0.6;
double cereal = 0.5;
double milk = 0.4;
double biscuit = 0.3;
static void initialize() {
  /* necessary to initialize Webots */
  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();
  
  camera = wb_robot_get_device("camera");
  compass = wb_robot_get_device("compass");
  assert(camera);
  wb_camera_enable(camera, time_step);
  wb_camera_recognition_enable(camera, time_step);
  wb_compass_enable(compass, time_step);
  
  ds_top = wb_robot_get_device("ds_top");
  so6 = wb_robot_get_device("so6");
  so7 = wb_robot_get_device("so7");
  so8 = wb_robot_get_device("so8");
  ds_front1 = wb_robot_get_device("ds_front1");
  ds_front2 = wb_robot_get_device("ds_front2");
  wb_distance_sensor_enable(ds_top, time_step);
  wb_distance_sensor_enable(so6, time_step);
  wb_distance_sensor_enable(so7, time_step);
  wb_distance_sensor_enable(so8, time_step);
  wb_distance_sensor_enable(ds_front1, time_step);
  wb_distance_sensor_enable(ds_front2, time_step);
  
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

static void recognize_object(){
    object = wb_camera_recognition_get_objects(camera);
    //int id = object->id;
    double *colors = object->colors;
    int x = object->position_on_image[0];
    int w = object->size_on_image[0];
    int h = object->size_on_image[1];
    if((x+w/2)<315 && x>100 && w>0 && h>100){
      if(colors[0] == redcan){
        targetshelf = 1;
        shelf_floor = 1;
        lift_degree = 0.29;
        finger_degree = 0.015;
        redcan = -1.0;
      }
      else if(colors[0] == beer){
        targetshelf = 2;
        shelf_floor = 1;
        lift_degree = 0.235;
        finger_degree = 0.014;
        beer = -1.0;
      }
      else if(colors[0] == greencan){
        targetshelf = 3;
        shelf_floor = 1;
        lift_degree = 0.28;
        finger_degree = 0.015;
        greencan = -1.0;
      }
      else if(colors[0] == water){
         targetshelf = 4;
         shelf_floor = 1;
         lift_degree = 0.195;
         finger_degree = 0.027;
         water = -1.0;
      }
      else if(colors[0] == biscuit){
        targetshelf = 1;
        shelf_floor = 2;
        lift_degree = 0.27;
        high = -0.2;
        i = 0;
        finger_degree = 0.023;
        biscuit = -1.0;
      }
      else if(colors[0] == jerry){
        targetshelf = 4;
        shelf_floor = 2;
        lift_degree = 0.3;
        high = -0.4;
        i = 0;
        finger_degree = 0.022;
        jerry = -1.0;
      }
      else if(colors[0] == milk){
        targetshelf = 3;
        shelf_floor = 2;
        lift_degree = 0.25;
        i = 2;
        high = -0.2;
        finger_degree = 0.02;
        milk = -1.0;
      }
      else if(colors[0] == cereal){
        targetshelf = 2;
        shelf_floor = 2;
        lift_degree = 0.25;
        i = 3;
        high = -0.4;
        finger_degree = 0.02;
        cereal = -1.0;
      }
      else{
      
      }

    }
}

static void movefirst(){
    start_time = wb_robot_get_time();
    end_time =wb_robot_get_time();
    while (end_time - start_time < linetime - putting_time - taking_time)
    {
      stepp();
      moveForwards(1.25);
      if(end_time - start_time>4.8-putting_time && end_time-start_time<linetime-4.8-putting_time)
      {
        recognize_object();
      }
      end_time =wb_robot_get_time();
      if(targetshelf != 0) break;
    }
    taking_time = end_time - start_time;
    stop(1.0);
}

static void compass_turn(){
  int go_time = 0;
  if(shelf == 2){
      //step(turntime1);
      while(true){
        direction = wb_compass_get_values(compass);
        if(-0.001<=direction[0] && direction[0]<=0.001 && direction[2]<=-0.999) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
      
    }
    else if(shelf == 3){
      while(true){
        direction = wb_compass_get_values(compass);
        if(direction[0]>=0.999 && -0.001<=direction[2] && direction[2]<=0.001) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
    }
    else if(shelf == 4){
      while(true){
        direction = wb_compass_get_values(compass);
        if(-0.001<=direction[0] && direction[0]<=0.001 && direction[2]>=0.999) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
    }
    else{
      //step(turntime2);
      while(true){
        direction = wb_compass_get_values(compass);
        if(direction[0]<=-0.999 && -0.001<=direction[2] && direction[2]<=0.001) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
      
    }
}
//鎷愯澶勮浆寮紝鏇存柊淇℃伅
static void turnleft(){
    int go_time = 0;
    turn(-0.2);
    if(shelf == 1){
      //step(turntime1);
      while(true){
        direction = wb_compass_get_values(compass);
        if(-0.001<=direction[0] && direction[0]<=0.001 && direction[2]<=-0.999) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
      
    }
    else if(shelf == 2){
      while(true){
        direction = wb_compass_get_values(compass);
        if(direction[0]>=0.999 && -0.001<=direction[2] && direction[2]<=0.001) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
    }
    else if(shelf == 3){
      while(true){
        direction = wb_compass_get_values(compass);
        if(-0.001<=direction[0] && direction[0]<=0.001 && direction[2]>=0.999) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
    }
    else{
      //step(turntime2);
      while(true){
        direction = wb_compass_get_values(compass);
        if(direction[0]<=-0.999 && -0.001<=direction[2] && direction[2]<=0.001) break;
        wb_robot_step(time_step);
        go_time += time_step;
      }
      
    }
    stop(1.0);
    putting_time = 0.0;   //鏇存柊鏃堕棿淇℃伅
    taking_time = 0.0;
    shelf = shelf + 1;    //鏇存柊shelf淇℃伅
    if(shelf == 5){
        shelf = 1;
    }
}

void movestraight(double second){
    stepp();
    moveForwards(1.25);
    step(second);
    stop(1.0);
    
}



//鍙栬揣鐗?
void taking_goods(double lift_degree, double finger_degree){
    int go = 0;
    turn(-pi);
    step(turntime1);
    stop(0.5);
    moveFingers(0.05);
    lift(lift_degree);
    value_front1 = wb_distance_sensor_get_value(ds_front1);
    value_front2 = wb_distance_sensor_get_value(ds_front2);
    while(value_front1 > 650 && value_front2 > 650){
        stepp();
        moveForwards(1.25);
        value_front1 = wb_distance_sensor_get_value(ds_front1);
        value_front2 = wb_distance_sensor_get_value(ds_front2);
        wb_robot_step(time_step);
        go += time_step;
    }
    stop(3.0);
    moveFingers(finger_degree+0.005);
    stop(2.0);
    moveFingers(0.04);
    stop(5.0);
    moveFingers(finger_degree);
    stop(5.0);
    lift(lift_degree-0.017);
    stop(5.0);
    while(go > 0){
        stepp();
        moveForwards(-1.25);
        wb_robot_step(time_step);
        go -= time_step;
    }
    stop(0.5);
    turn(0.2);
    compass_turn();
    stop(0.5);
}

//鎶婅揣鐗╂斁鍒拌揣鏋朵笂

void putting_goods_one(double lift_degree){
  int i = 0;
  int go = 0;
  moveForwards(1.25);
  step(2.76);
  stop(1.0);
  value6 = wb_distance_sensor_get_value(so6);
  value7 = wb_distance_sensor_get_value(so7);
  value8 = wb_distance_sensor_get_value(so8);
  printf("%10.3f\n",value6+value7+value8);
  while(value6+value7+value8>2696 || value6+value7+value8<1900){
    stepp();
    moveForwards(1.25);
    step(2.04);
    stop(1.0);
    i=i+1;
    
    value6 = wb_distance_sensor_get_value(so6);
    value7 = wb_distance_sensor_get_value(so7);
    value8 = wb_distance_sensor_get_value(so8);
    printf("%10.3f\n",value6+value7+value8);
  }
  
  putting_time = 2.76+i*2.04;
  
  turn(pi);
  step(turntime2);
  //lift(0.03);
  stop(0.5);
  //moveForwards(1.25);
  //step(2.1);
  
  value_top = wb_distance_sensor_get_value(ds_top);
  //printf("%10.3f\n", value_top);
  while(value_top > 720.0){
    stepp();
    moveForwards(1.25);
    value_top = wb_distance_sensor_get_value(ds_top);
    //printf("%10.3f\n", value_top);
    wb_robot_step(time_step);
    go += time_step;
  }
  
  stop(1.0);
  lift(lift_degree);
  stop(1.0);
  moveFingers(0.035);
  stop(2.0);
  //moveForwards(-1.25);
  //step(2.1);
  
  while(go > 0){
    stepp();
    moveForwards(-1.25);
    wb_robot_step(time_step);
    go -= time_step;
  }
  moveFingers(0.04);
  stop(0.5);
  lift(0.2);
  turn(-0.2);
  compass_turn();
  stop(0.5);
  
}

void putting_goods_two(double lift_degree, double high){
    int go = 0;

    moveForwards(1.25);
    step(2.76 + i*2.04);
    stop(1.0);
    putting_time = 2.76 + i*2.04;
    turn(pi);
    step(turntime2);
 
    stop(0.5);
    lift(high + lift_degree - 0.017);
    stop(5.0);
    value_top = wb_distance_sensor_get_value(ds_top);

    while(value_top > 720.0){
      stepp();
      moveForwards(1.25);
      value_top = wb_distance_sensor_get_value(ds_top);

      wb_robot_step(time_step);
      go += time_step;
    }
        
    stop(1.0);
    lift(high + lift_degree);
    stop(1.0);
    moveFingers(0.035);
    stop(2.0);
    while(go > 0){
      stepp();
      moveForwards(-1.25);
      wb_robot_step(time_step);
      go -= time_step;
    }
    moveFingers(0.04);
    stop(0.5);
    lift(0.2);
    turn(-0.2);
    compass_turn();
    stop(0.5);

        
}
//鏁翠釜鍙?鏀捐繃绋?
void wholeprocess(){
    while(true){
        movefirst();
        if(targetshelf != 0) break;
        turnleft();
    }
    
    taking_goods(lift_degree, finger_degree);
    
    movestraight(linetime - taking_time - putting_time);
    turnleft();
    
    while(shelf != targetshelf){
        movestraight(linetime);
        turnleft();
    }
    if(shelf_floor == 1){
        putting_goods_one(lift_degree);
    }
    else if(shelf_floor == 2){
        putting_goods_two(lift_degree, high);
    }
    targetshelf = 0;
    shelf_floor = 0;
    i = 0;
}

int main() {
  
  initialize();
  lift(0.2);
  passive_wait(2.0); 

  
  /*while (true) {
    stepp();
    //wholeprocess();
  
  }*/
  
  while(true){

    wholeprocess();
    //movefirst();
    //turnleft();
  }
  
  

  wb_robot_cleanup();
  

  return 0;
}