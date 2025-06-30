#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor rf = motor(PORT11, ratio6_1, false);
motor intake = motor(PORT12, ratio6_1, false);
motor rb = motor(PORT13, ratio6_1, false);
motor turret = motor(PORT14, ratio6_1, false);
motor lb = motor(PORT15, ratio6_1, true);
motor lf = motor(PORT16, ratio6_1, true);
motor indexer = motor(PORT17, ratio6_1, false);
motor flywheel1 = motor(PORT18, ratio6_1, true);
motor flywheel2 = motor(PORT19, ratio6_1, false);
motor hood = motor(PORT20, ratio6_1, true);
encoder left_tracker = encoder(Brain.ThreeWirePort.E);
encoder right_tracker = encoder(Brain.ThreeWirePort.G);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}