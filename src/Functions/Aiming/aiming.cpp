#include "aiming.h"
#include "vex.h"

#include "../../RobotParameters/parameters.h"
#include "../../Structures/structures.h"


// Task to aim the turret at the goal based on the calculated angle from the latency compensation. 
int Aim_Turret()  
{
  turret.setBrake(brake);
  const double kp = 0.7, kd = 0.6, ki = 0.000001, ks = 2.0, loop_speed = 10.0, min_speed = 10.0, deadzone = 10.0;                                     
  double p = 0.0, d = 0.0, i = 0.0, err = 0.0, last_err = 0.0, speed = 0.0, last_speed = 0.0;;

  while(true)
  {
    if(aim_at_goal)                                                                                                                                   // If robot is set to aim
    {
      double tar_angle = tur.target_angle * 85.0 / 14.0;                                                                                              // Account for gear ratio on turret
      if(tur.target_angle <= turret_max && tur.target_angle >= turret_min)                                                                            
      {
        if(turret.rotation(deg) > tar_angle + deadzone || turret.rotation(deg) < tar_angle - deadzone)                                                // If the target angle is within the range of motion
        {
          double cur_angle = turret.rotation(deg);
          
          err = tar_angle - cur_angle;                                                                                                                // Simple PID calculations 
          p = err*kp;
          d = ((err - last_err)/loop_speed) * kd;
          i += (((err + last_err)/2.0) * loop_speed) * ki;

          double ff_speed = tur.speed * 85.0/14.0;
          
          ff_speed = (fabs(ff_speed) < min_speed ? (ff_speed > 0 ? min_speed : -min_speed) : ff_speed);                                                // Feed forwards speed calculated by change in target turret angle

          speed = tur.speed*85.0/14.0/1.5 + p + i + d;      

          if(speed < 0.0)                                                                                                                              // Creates deadzone to prevent oscillation
          {
            if(speed > -min_speed) {speed = -min_speed;}
            if(turret.rotation(deg) <= tar_angle - deadzone) {speed = 0.0;}
          }
          else if(speed > 0.0)
          {
            if(speed < min_speed) {speed = min_speed;}
            if(turret.rotation(deg) > tar_angle + deadzone) {speed = 0.0;}
          }
          else {speed = 0.0;}
        }
        else 
        {
          speed = 0.0;
          i = 0.0;
        }
        turret.spin(fwd, speed + (fabs(speed) > fabs(last_speed) ? (fabs(speed - last_speed) > 30.0 ? (speed - last_speed) * ks : 0.0):0.0), rpm);    // Moves the turret
      }
    }
    last_err = err;
    last_speed = speed;
    wait(loop_speed, msec);
  }
  return 0;
}


// Task to literally just spin the flywheel at max speed with button toggle, no PID 
int Flywheel_Control()
{

  bool spinning = false; 

  while(true)
  {
    if(Controller1.ButtonR2.pressing())
    {
      while(Controller1.ButtonR2.pressing())
      {
        wait(10, msec);
      }
      if(spinning)
      {
        flywheel1.stop();
        flywheel2.stop();
        spinning = false;
      }
      else 
      {
        flywheel1.spin(fwd, 12, volt);
        flywheel2.spin(fwd, 12, volt);
        spinning = true;
      }
    }
    wait(10, msec);
  }

  return 0;
}


// Task to control the hood angle
int Hood_Control()
{
  while(1)
  {
    double hood_ang = -0.0352039 * powf(tur.dis+14.0,2) + 9.32211 * (tur.dis+14.0) + -256.183; // Used quadratic regression to create line of best fit between distance and hood angle
    if(tur.dis > 100.0) {hood_ang = 365.0;}                                                    // Accounts for the range of the hood
    if(hood_ang < 5.0) {hood_ang = 5.0;}
    hood.spinTo(hood_ang, deg, 300.0, rpm);                                                    // Spins the hood
    wait(10, msec);
  }
  return 0;
}