#include "vex.h"

#include "RobotParameters/parameters.h"
#include "Structures/structures.h"
#include "Functions/General/general.h"
#include "Functions/Reading-From-USB/reading.h"
#include "Functions/Latency-Compensation/latency.h"
#include "Functions/Aiming/aiming.h"

using namespace vex;
using namespace std;

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
    else 
    {
      intake.stop();
      indexer.stop();
    }

    wait(5, msec);                                        // Obligatory
  }
}