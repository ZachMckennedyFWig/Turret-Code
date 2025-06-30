#include "general.h"
#include "vex.h"

/*
Basic functions for moving around the robot and debugging.  
*/

using namespace vex;
 
// Fucntion to get the drive encoder values in the Encoder struct format
Encoder get_drive_encoder()
{
  Encoder temp;
  temp.left = left_tracker.rotation(rotationUnits::deg);
  temp.right = right_tracker.rotation(rotationUnits::deg);
  return temp;
}

// Function to print the coords that the camera is seeing
void print_coords(int x, int y)
{
  // Prints the coordinates of the objects on the brain screen
  Brain.Screen.printAt(0, 10, "Camera Detection");
  Brain.Screen.printAt(0, 40, "X: %03d", x);
  Brain.Screen.printAt(0, 70, "Y: %03d", y);
}

// Moves the left side of the drive at the given voltage, Max Voltage is 12.0
void move_left_vol(double vol)
{
	lf.spin(fwd, vol*.12, volt);
	lb.spin(fwd, vol*.12, volt);
}


// Moves the right side of the drive at the given voltage
void move_right_vol(double vol)
{
	rf.spin(fwd, vol*.12, volt);
	rb.spin(fwd, vol*.12, volt);
}


// Sets the entire drive to coast braking for driver control
void set_drive_coast()
{
	lf.setStopping(coast);
	lb.setStopping(coast);
  rf.setStopping(coast);
	rb.setStopping(coast);
}