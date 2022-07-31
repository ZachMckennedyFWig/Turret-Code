#include "parameters.h"

// Sorry to anyone that cares about not having globals, this is just easiest to understand imo.... 

const int buffer_size = 6;                        // Stores the buffer size of the serial data being recieved from the Raspberry pi

const double camera_y_res = 300.0;                // Stores the number of pixels in the Y direction on the camera

const double camera_x_res = 300.0;                // Stores the amount of pixels in the X direction on the camera

const double camera_horizontal_fov = 55.0;        // Stores the camera's up and down FOV in degrees

const double camera_vertical_fov = 45.0;          // Stores the camera's left and right FOV in degrees

const double target_height = 62.5;                // Stores the height of the goal/target the vision system is looking at in inches

const double camera_angle = 45.0;                 // Stores the camera angle relative to the ground 

const double camera_height = 13.75;                // Stores the height of the middle of the camera from the floor

const double fps = 20.0;                          // Stores the set FPS of the camera running the trained model

const double wheel_diameter = 3.25;               // Stores the driven chassis wheel diameter or the tracking wheel diameter

const double wheel_ratio = 20.0/10.0;             // Stores the ratio between either the motor and the driven wheel or the encoder to the tracking wheel 

const double ticks_per_rev = 360.0 * wheel_ratio; // Stores the amount of ticks per revolution of the encoder or motor

const double chassis_width = 10.75;               // Stores the width between the wheels that will be tracking position are

const double odom_update_speed = 25.0;            // Stores the speed in ms the odometry will update 

const double turret_max = 138.0;                  // Max distance the turret can turn counter-clockwise

const double turret_min = -138.0;                 // Max distance the turret can turn clockwise

bool aim_at_goal = false;                         // Controls if task should be aiming at the goal - lazy global because I like doing it this way. 

const double PI = 3.1415926;
