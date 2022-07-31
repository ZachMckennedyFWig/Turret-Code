#ifndef structures_H
#define structures_H

// Yea this is really bad since they need to be global so I can't externally declare them.... oops! should have used classes. 

// Structure to store robot states at specific frames
struct Position
{
	double x, y, theta, turret_ang, target_ang, target_dis;
};

Position latency_positions[3]; 

// Structure to store the encoder values
struct Encoder
{
  double left, right;
};

struct Camera
{
  int x,y;
}cam;

struct Turret
{
  double speed = 0.0;
  double target_angle = 0.0;
  double dis = 0.0;
}tur;


#endif