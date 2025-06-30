using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor turret;
extern motor lf;
extern motor lb;
extern motor rf;
extern motor rb;
extern motor intake;
extern motor indexer;
extern motor flywheel1;
extern motor flywheel2;
extern motor hood;

extern encoder left_tracker;
extern encoder right_tracker;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );