#include "latency.h"
#include <math.h>
#include "vex.h"

#include "../RobotParameters/parameters.h"
#include "../General/general.h"

using namespace vex;


// Returns the angle from the center of the camera to the goal
double get_vert_angle_to_target(double y)
{
  // y = y position of target in camera
  return -(y - (camera_y_res/2.0))*(camera_vertical_fov/camera_y_res);
}


// Returns the angle from the center of the camera to the goal
double get_hor_angle_to_target(double x)
{
  // x = x position of target in camera
  return (x - (camera_x_res/2.0))*(camera_horizontal_fov/camera_x_res);
}


// Returns the distance from the goal given the pixel value of the middle of the target as the input
double get_dis_from_goal(double y)
{
  // y = y position of target in camera
  return (target_height - camera_height)/tanf((camera_angle + get_vert_angle_to_target(y)) * 3.14/180.0);
}


// Function to calculate the number of inches the robot has moved based on the encoder ticks
double ticks_to_inches(double ticks)
{
  return (ticks * PI * wheel_diameter) / ticks_per_rev;
}


void odom(double last_left, double last_right, double left_ticks, double right_ticks)
{
  /*
    Function to calculate the cartesian displacement from the given "frame" based on the drives encoder change (using odom).

    This is set up like this because each buffer frame will have started from a different "referance frame" meaning that 
      each cartesian displacement will need to be independant since they could all be at different angles. This is required 
      to avoid using a global odometry position and instead rely on odometry over short intervals for feed forwards and use 
      the feed back from the camera to accound for any error that begins to accumulate. This is a more robust solution 
      because it will be resistance to wheel slipping, getting pushed, odometry drift after long periods from imperfect 
      manufacturing. 

    frame - Position struct of coordinate frame we are going to calcualte odom on
    left_ticks - Amount of ticks the left encoders have read
    right_ricks - Amount of ticks the right encoders have read
  */

  double dist_left = ticks_to_inches(left_ticks - last_left),   // Stores the distance in inches of the left side
         dist_right = ticks_to_inches(right_ticks - last_right), // Stores the distance in inches of the right side
         alpha,                                     // Stores the center distance in inches of the drive
         h,                                         // Stores the hypotenuse length of the right triangle made
         temp_sin,                                  // For readability, intermediate trig math
         temp_cos,                                  // --
         delta_x,                                   // Stores the cartesian X change
         delta_y;                                   // Stores the cartesian y change
  
  if(dist_left != dist_right)                                           // Edge case, prevents divide by zero if both sides travel the exact same distance. 
  {
    alpha = (dist_right - dist_left)/chassis_width;                     // Calcualtes the angle the robot turned over the movement

    glob_pos.theta += alpha * 180.0 / PI;                               // Stores the angle change in degrees
    h = 2.0*((dist_left/alpha) + (chassis_width/2.0))*sinf(alpha/2.0);  // Calculates the hypotenuse of the triangle the robot traveled on

    temp_cos = cosf(alpha/2.0 + glob_pos.theta * PI / 180.0);           // Trig to calcualte cos of cartesian triangle moved
    temp_sin = sinf(alpha/2.0 + glob_pos.theta * PI / 180.0);           // Trig to calcualte sin of cartesian triangle moved

    delta_x = h*temp_cos;                                                // Calculates X displacement
    delta_y = h*temp_sin;                                               // Calcualtes Y displacement

    glob_pos.x += delta_x;                                                  // Adds displacement to the position struct
    glob_pos.y += delta_y;                                                  // Adds displacement to the position struct
  }
  else
  {
    glob_pos.x += dist_right * cosf(glob_pos.theta * PI / 180.0);           // Adds displacement if perfectly straight line movement
    glob_pos.y += dist_right * sinf(glob_pos.theta * PI / 180.0);           // --
  }
  //Brain.Screen.printAt(0, 15, "x: %f", glob_pos.x); 
  //Brain.Screen.printAt(0, 30, "y: %f", glob_pos.y);   
  //Brain.Screen.printAt(0, 45, "theta: %f", glob_pos.theta);                                                       
}

void update_tar(double angle_to_target, double dis_from_goal, double elps_time)
{
  // Initialize angle to aim and third side mag
  double angle_to_aim = 0.0;                                                                                                    
  double third_side_mag = dis_from_goal; // Set to this in case there is no movement

  // Prevents edge case of not seeing the goal on initial start up by waiting until it has found the goal at least once 
  if(dis_from_goal != 0.0)                                                                                                      
  {
    // Variables to store the correct X and Y cartesian coordinates of the target  
    double tar_x, tar_y;                                                                                                                                                                        

    // Calculates the targets position relative to the current robot position via vector addition. 
    //  i.e adds the vectors of the robot displacement and the cameras calculates distances 
    tar_x = dis_from_goal * cosf(angle_to_target*PI/180.0) - glob_pos.x;
    tar_y = dis_from_goal * sinf(angle_to_target*PI/180.0) - glob_pos.y; 
    // Calculate the distance from the robots current position to the goal. 
    third_side_mag = sqrtf(powf(tar_x, 2) + powf(tar_y, 2));

    // In order to calculate the angle the turret needs to aim at now, a virtual unit header will be created as a vector pointing
    //  towards the front of the robot, the angle then calculated between that unit header and the vector from the robots current
    //  position to the goal, as calculated above, will give the angle the turret needs to be pointing towards. 

    // Create Virtual unit heading
    double v_x = cosf(glob_pos.theta*PI/180.0);
    double v_y = sinf(glob_pos.theta*PI/180.0);

    // Calculate the angle between the virtual heading and the target (dot product)
    double real_ang = fabs(acosf((tar_x*v_x + tar_y*v_y)/(third_side_mag)) * 180.0 / PI);
    // Using the cross product we can determing if the angle between these vectors is positive or negative, 
    //  the math is just a 2x2 matrix determinant of the x and y values for the virtual point an the target. 
    //  
    // The cross product lets us do this because of the right hand rule, if it is positive, the z axis points
    //  upwards and the angle is positive. If the cross product is negative the z axis is pointing downwards 
    //  and the angle is negative. 
    double cross = v_x*tar_y - tar_x*v_y;

    // If the Target is to the left 
    if(cross > 0.0){angle_to_aim = real_ang;}
    else if(cross < 0.0){angle_to_aim = -real_ang;}
    else{angle_to_aim = 0.0;}

    // Estimate the speed the turret will need to spin at. 
    tur.speed = ((angle_to_aim - tur.target_angle) * fps * 60.0)/360.0; 
    Brain.Screen.printAt(0, 180, "tur speed %f", tur.speed);
    tur.target_angle = angle_to_aim;
    tur.dis = third_side_mag;

    // Now calcualte the angle the turret should be aiming at given the balls time of flight
    //  and the tangential velocity to the goal. 

    // Step 1: calculate the tangential velocity:
    double vel_mag = sqrtf(powf(glob_pos.x, 2) + powf(glob_pos.y, 2)); 
    double tan_vel = vel_mag * sinf(angle_to_aim * PI / 180.0);

    // Step 2: Calculate the time of flight of the ball:
    double tof = third_side_mag / (12.5 * 12.0); // 12.5 * 12.0 is the velocity of the ball in in/sec

    // Step 3: calculate the distance the ball will travel in that time period: 
    double tan_dis = tan_vel * tof;

    // Step 4: Calculate the angle that the tangent displacement will cause. 
    double tof_ang = atanf(tan_dis/third_side_mag) * 180.0 / PI; 

    // Step 5: Add the offset to the aim angle
    if(glob_pos.x > 0.0) {tur.tof_ang = angle_to_aim + tof_ang;}
    else if(glob_pos.x <= 0.0) {tur.tof_ang = angle_to_aim - tof_ang;}
  }
}


int Latency_Comp()
{
  /*
    Task to create a buffer of referance frames with the robots position in order to account for every frames latency. 

    This task takes the video stream and uses the pinhole model of a camera in order to calcualte the distances and angles to the goal.
      However, there is a small latency in the video stream (roughly 50ms). To account for this, with every single frame created a referance 
      frame for odometry is also created. This frame is updated until the current frame that is being sent to the brain is alligned with the orgin
      of the referance frame for the robots position. This referance frame is than pulled out, and feed forwards control based on where the robot was when the image was 
      taken vs where the robot is now can be used to ajust the camera to the correct position and that referance frame can be deleted. 
  */

  // Initialize Encoders 
  double cur_left=0.0, cur_right=0.0, last_left, last_right;

  // Variable to store the time delay needed for each frame
  const double frame_timing = 1000.0/fps;
  const int num_loop = int(frame_timing/odom_update_speed);

  //initialize camera
  cam.x = 150.0;
  cam.y = 150.0; 

  glob_pos.target_ang = 0.0;
  glob_pos.target_dis = 0.0;
  tur.dis = 0.0;
  tur.target_angle = 0.0;

  while(true)
  {
    glob_pos.x = 0.0;
    glob_pos.y = 0.0;
    glob_pos.theta = 0.0;
    double tur_ang = turret.position(deg) * 14.0/85.0;
    Brain.Screen.printAt(150, 210, "%f", tur_ang);

    for(int i = 0; i < num_loop; i++)
    {
      last_left = cur_left;
      last_right = cur_right;
      // Update the position with odometry and aim until the frame is captured.
      wait(odom_update_speed, msec);
      //wait(2000, msec);
      cur_left = left_tracker.position(deg);
      cur_right = right_tracker.position(deg);
      odom(last_left, last_right, cur_left, cur_right);
      
      // Odom updated, Now lets set a target to aim at. 
      double dis_from_goal = glob_pos.target_dis;
      //Brain.Screen.printAt(0, 15, " Dis From Goal %f", dis_from_goal);
      double angle_to_target = glob_pos.target_ang;
      //Brain.Screen.printAt(0, 30, " Ang To Target %f", angle_to_target);

      update_tar(angle_to_target, dis_from_goal, (double(i+1)*odom_update_speed));
    }

    // Image from a while ago
    double cam_y = double(cam.y), cam_x = double(cam.x);        
    
    if(cam_y < 300.0 && cam_x < 300.0)                                                                                              // If the camera is curently tracking the goal
    {
      Brain.Screen.printAt(0, 100, "Ang %f", get_hor_angle_to_target(cam_x));
      double latant_dis = get_dis_from_goal(cam_y);                                                                                   // Gets the distance from the camera to the target along the floor
      double latant_ang = get_hor_angle_to_target(cam_x) + tur_ang;//old tur pos;                                             // Gets the angle from the X axis to the target
      update_tar(latant_ang, latant_dis, double(num_loop) * odom_update_speed);
    }
    glob_pos.target_dis = tur.dis;                                                                                // Use the last calculated distance from the goal instead of the cameras measured distance
    glob_pos.target_ang = tur.target_angle;                                                                              // Use the last calculated angle to aim at as the angle to the goal instead of the measured camera angle  
  }
  return 0;
};