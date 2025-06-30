#ifndef latency_H
#define latency_H

#include "../Structures/structures.h"

double get_vert_angle_to_target(double y);
double get_hor_angle_to_target(double x);
double get_dis_from_goal(double y);
double ticks_to_inches(double ticks);
Position odom(Position frame, double left_ticks, double right_ticks);
int Latency_Comp();

#endif