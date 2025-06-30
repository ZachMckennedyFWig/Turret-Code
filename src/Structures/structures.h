#ifndef structures_H
#define structures_H

// Yea this is really bad since they need to be global so I can't externally declare them.... oops! should have used classes. 

// Structure to store robot states at specific frames
struct Position
{
	double x, y, theta, turret_ang, target_ang, target_dis;
};

// Structure to store the encoder values
struct Encoder
{
  double left, right;
};

struct Camera
{
  int x,y;
};

struct Turret
{
  double speed;
  double target_angle;
  double dis;
  double tof_ang;
};

extern Turret tur;
extern Camera cam; 
extern Position latency_positions[3];
extern Position glob_pos;

#endif