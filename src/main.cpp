#include "vex.h"

#include "RobotParameters/parameters.h"
#include "Structures/structures.h"
#include "General/general.h"
#include "Reading-From-USB/reading.h"
#include "Latency-Compensation/latency.h"
#include "Aiming/aiming.h"

using namespace vex;
using namespace std;

int trolli()
{
  double alpha, h, temp_cos, temp_sin, delta_x, delta_y;
  double theta=0.0, x=0.0, y=0.0, llast=0.0, rlast=0.0,
          ticks_left =0.0, ticks_right=0.0, dist_left=0.0, dist_right=0.0;
  while(true)
  {
    ticks_left = left_tracker.rotation(rotationUnits::deg);
    ticks_right = right_tracker.rotation(rotationUnits::deg);

    dist_left = ticks_to_inches(ticks_left - llast);
    dist_right = ticks_to_inches(ticks_right - rlast);

    if(dist_left != dist_right)                                           // Edge case, prevents divide by zero if both sides travel the exact same distance. 
    {
      alpha = (dist_right - dist_left)/chassis_width;                     // Calcualtes the angle the robot turned over the movement

      theta += alpha * 180.0 / PI;                               // Stores the angle change in degrees
      h = 2.0*((dist_left/alpha) + (chassis_width/2.0))*sinf(alpha/2.0);  // Calculates the hypotenuse of the triangle the robot traveled on

      temp_cos = cosf(alpha/2.0 + theta * 3.1515 / 180.0);           // Trig to calcualte cos of cartesian triangle moved
      temp_sin = sinf(alpha/2.0 + theta * 3.1415 / 180.0);           // Trig to calcualte sin of cartesian triangle moved

      delta_x = h*temp_cos;                                                // Calculates X displacement
      delta_y = h*temp_sin;                                               // Calcualtes Y displacement

      x += delta_x;                                                  // Adds displacement to the position struct
      y += delta_y;                                                  // Adds displacement to the position struct
    }
    else
    {
      x += dist_right * cosf(theta * 3.1415 / 180.0);           // Adds displacement if perfectly straight line movement
      y += dist_right * sinf(theta * 3.1415 / 180.0);           // --
    }  
    Brain.Screen.printAt(0,45, "Theta %f", theta);
    Brain.Screen.printAt(0,15, "x %3.5f", x);
    Brain.Screen.printAt(0,30, "y %3.5f", y);
    llast = ticks_left;
    rlast = ticks_right;
    wait(15, msec);
  }

  return 0;
}


int main() 
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Starts tasks
  task t (Read_Loop);
  task n (Latency_Comp);
  task p (Aim_Turret);
  task q (Flywheel_Control);
  task g (Hood_Control);
  //task o (trolli);

  // Makes sure everything is in coast mode
  flywheel1.setBrake(coast);
  flywheel2.setBrake(coast);
  intake.setBrake(coast);
  indexer.setBrake(coast);

  while(true)
  {
    aim_at_goal = true;                                   // Always set to just aim at the goal but this could be toggled. 

    move_left_vol(Controller1.Axis3.position(percent));   // Drive code
    move_right_vol(Controller1.Axis2.position(percent));

    if(Controller1.ButtonR1.pressing())                   // Intake code
    {
      intake.spin(fwd, 12, volt);
      indexer.spin(fwd, 12, volt);
    }
    else if(Controller1.ButtonR2.pressing())
    {
      intake.spin(fwd, 12, volt);
    }
    else
    {
      intake.stop();
      indexer.stop();
    }

    wait(15, msec);                                        // Obligatory
  }
}