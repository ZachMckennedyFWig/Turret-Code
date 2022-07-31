#include "latency.h"
#include <math.h>
#include "vex.h"

#include "../../RobotParameters/parameters.h"
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


Position odom(Position frame, double left_ticks, double right_ticks)
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

  double dist_left = ticks_to_inches(left_ticks),   // Stores the distance in inches of the left side
         dist_right = ticks_to_inches(right_ticks), // Stores the distance in inches of the right side
         alpha,                                     // Stores the center distance in inches of the drive
         h,                                         // Stores the hypotenuse length of the right triangle made
         temp_sin,                                  // For readability, intermediate trig math
         temp_cos,                                  // --
         delta_x,                                   // Stores the cartesian X change
         delta_y;                                   // Stores the cartesian y change
  
  Position temp = frame;                            // Temp "Position" struct that will be returned                           

  if(dist_left != dist_right)                                           // Edge case, prevents divide by zero if both sides travel the exact same distance. 
  {
    alpha = (dist_right - dist_left)/chassis_width;                     // Calcualtes the angle the robot turned over the movement

    temp.theta += alpha * 180.0 / 3.1415;                               // Stores the angle change in degrees
    h = 2.0*((dist_left/alpha) + (chassis_width/2.0))*sinf(alpha/2.0);  // Calculates the hypotenuse of the triangle the robot traveled on

    temp_cos = cosf(alpha/2.0 + temp.theta * 3.1515 / 180.0);           // Trig to calcualte cos of cartesian triangle moved
    temp_sin = sinf(alpha/2.0 + temp.theta * 3.1415 / 180.0);           // Trig to calcualte sin of cartesian triangle moved

    delta_x = h*temp_cos;                                                // Calculates X displacement
    delta_y = h*temp_sin;                                               // Calcualtes Y displacement

    temp.x += delta_x;                                                  // Adds displacement to the position struct
    temp.y += delta_y;                                                  // Adds displacement to the position struct
  }
  else
  {
    temp.x += dist_right * cosf(temp.theta * 3.1415 / 180.0);           // Adds displacement if perfectly straight line movement
    temp.y += dist_right * sinf(temp.theta * 3.1415 / 180.0);           // --
  }  
  return temp;                                                          // Returns the new positon
}


int Latency_Comp()
{
  /*
    Task to create a buffer of referance frames with the robots position in order to account for every frames latency. 

    This task takes the video stream and uses the pinhole model of a camera in order to calcualte the distances and angles to the goal.
      However, there is a small latency in the video stream (roughly 150ms). To account for this, with every single frame created a referance 
      frame for odometry is also created. This frame is updated until the current frame that is being sent to the brain is alligned with the orgin
      of the referance frame for the robots position. This referance frame is than pulled out, and feed forwards control based on where the robot was when the image was 
      taken vs where the robot is now can be used to ajust the camera to the correct position and that referance frame can be deleted. 
  */

  // Initialize Encoders 
  Encoder last_dis = {0.0,0.0};

  // Variable to store the time delay needed for each frame
  const double frame_timing = 1000.0/fps;

  while(true)
  {

    double cam_y = cam.y, cam_x = cam.x;                                                                                          // Gets the targets position from the camera   
    
    for(int i = 0; i < int(frame_timing/odom_update_speed); i++)                                                                  // Loop to do odom calculations until the Fram and picture timings allign
    {
      wait(odom_update_speed, msec);                                                                                              // Wait the set amount of time

      Encoder temp_ticks = get_drive_encoder();                                                                                   // Get the current encoder position 

      for(int j = 0; j < int(latency_frames); j++)                                                                                // Loop over each buffering position
      {
        latency_positions[j] = odom(latency_positions[j], temp_ticks.left - last_dis.left, temp_ticks.right - last_dis.right);    // Update the odom position for the buffering position and its respective frame
      }
      last_dis = temp_ticks;                                                                                                      // Store the current distance the robot has travelled 
    }

    Position current_position = latency_positions[0];                                                                             // Get the current robots position vector from when the frame was taken

    double dis_from_goal,                                                                                                         
           angle_to_target;

    if(cam_y != 999 && cam_x != 999)                                                                                              // If the camera is curently tracking the goal
    {
      dis_from_goal = get_dis_from_goal(cam_y);                                                                                   // Gets the distance from the camera to the target along the floor
      angle_to_target = get_hor_angle_to_target(cam_x) + current_position.turret_ang;                                             // Gets the angle from the X axis to the target
    }
    else                                                                                                                          // Else, the camera lost the target
    {
      dis_from_goal = current_position.target_dis;                                                                                // Use the last calculated distance from the goal instead of the cameras measured distance
      angle_to_target = current_position.target_ang;                                                                              // Use the last calculated angle to aim at as the angle to the goal instead of the measured camera angle 
    }

    double angle_to_aim = 0.0;                                                                                                    // Creates variable for the actual angle the turret should be aiming at
    double third_side_mag = dis_from_goal;

    if(dis_from_goal != 0.0)                                                                                                      // Prevents edge case of not seeing the goal on initial start up by waiting until it has found the goal at least once 
    {
      double target_x, target_y;                                                                                                  // Variables to store the correct X and Y cartesian coordinates of the target                                                                        

      target_x = dis_from_goal * cosf(angle_to_target*PI/180.0);                                                                  // Calculates the correct cartesian value of the goal
      target_y = dis_from_goal * sinf(angle_to_target*PI/180.0);                                                                  // --
      
      double angle_between_vectors, angle_to_drive = 0.0;                                                                         // Variable to store the angle between the position and target vectors

      double movement_mag = sqrtf(powf(current_position.x, 2) + powf(current_position.y,2));                                      // Calculates the magnitude of the position vector 

      if(movement_mag != 0.0)                                                                                                     // If robot has moved
      {

        if(current_position.x >= 0.0)                                                                                             // Robot was either moving forwards or backwards. (yea i know)
        {
          angle_to_drive = 0.0;
        }
        else
        {
          if(current_position.y >= 0.0)
          {
            angle_to_drive = -180.0;
          }
          else 
          {
            angle_to_drive = -180.0;
          }
        }

        double angle_between_calc;

        if(angle_to_target*angle_to_drive >= 0.0) {angle_between_calc = fabs(angle_to_target-angle_to_drive);}                    // Turns angle from 0 to 360 into 0 to 180 and 0 to -180
        else {angle_between_calc = fabs(angle_to_target) + fabs(angle_to_drive);}

        if(angle_between_calc > 180.0) {angle_between_vectors = 360.0 - angle_between_calc;}
        else {angle_between_vectors = angle_between_calc;}

        third_side_mag = sqrtf(powf(current_position.x - target_x, 2) + powf(target_y, 2));
      }
      else
      {
        angle_between_vectors = 0.0;                                                                                              // If the robot hasn't moved, the vector does not exist and therefor there is no angle between the vectors
      }
      

      double angle_to_ajust = 0.0;  

      if(angle_between_vectors != 0.0)                                                                                            // Because the law of cos always returns positive acute angle between 2 vectors,
      {                                                                                                                           //  this determins the true angle of the final calculated angle
        if(angle_between_vectors > 90.0)                                                                                         
        {
          angle_to_ajust = (asinf((dis_from_goal*sinf(angle_between_vectors*PI/180.0))/third_side_mag)) * 180.0/PI;                
        }
        else
        {
          angle_to_ajust = 180.0 - (asinf((dis_from_goal*sinf(angle_between_vectors*PI/180.0))/third_side_mag)) * 180.0/PI;
        }
      }

      double cross_product = current_position.x * target_y;                                                                       // Fancy trick to determins if the angle is clockwise or counter-clockwise based on this being negative or positive 
                                                                                                                                  //  by using the cross product. 
      if(movement_mag != 0.0 && angle_to_ajust != 0.0)                                                                             
      {
        if(current_position.x >= 0.0)
        {
          if(cross_product >= 0.0) {angle_to_aim = 180.0 - angle_to_ajust;}
          else {angle_to_aim = -180.0 + angle_to_ajust;}
        }
        else                                                                                                                      // This will account for the acute angle from the law of cos and ajust to the correct angle.
        {
          if(cross_product >= 0.0) {angle_to_aim = -angle_to_ajust;}
          else {angle_to_aim = angle_to_ajust;}
        }
      }
      else
      {
        angle_to_aim = angle_to_target;
      } 
    }

    double final_angle;
    bool lost = false; 

    final_angle = angle_to_aim - current_position.theta;                                                                          // Calculates the final angle that the turret needs to aim from the angle offset of movement and turning. 

    Brain.Screen.printAt(0,190, "angle final %f", final_angle);

    tur.speed = ((final_angle - tur.target_angle) * fps * 60.0)/360.0;                                                            // Calculate the speed in RPM the turret needs to be moving
    tur.target_angle = (lost == false ? final_angle : final_angle / 2.0);                                                         // Sets the angle the turret should aim at
    tur.dis = dis_from_goal;

    for(int n = 0; n < int(latency_frames-1.0); n++)
    {
      latency_positions[n] = latency_positions[n+1];
    }

    Position blank;                                                                                                               // I forgot why i did this instead of {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} 
    blank.x = 0.0;
    blank.y = 0.0;                                                                                                                // but i am not gonna touch it.... 
    blank.target_ang = 0.0;
    blank.target_dis = 0.0;
    blank.theta = 0.0;
    blank.turret_ang = 0.0;

    latency_positions[int(latency_frames-1.0)] = blank;                                                                           // Remembering old stuff in case frames get missed 
    latency_positions[int(latency_frames-1.0)].turret_ang = turret.rotation(deg) * 14.0 / 85.0;
    latency_positions[int(latency_frames-1.0)].target_ang = final_angle;
    latency_positions[int(latency_frames-1.0)].target_dis = third_side_mag; 
  }
  return 0;
}