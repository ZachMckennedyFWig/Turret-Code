#ifndef latency_H
#define latency_H

#include "../../Structures/structures.h"

extern double get_vert_angle_to_target(double y);
extern double get_hor_angle_to_target(double x);
extern double get_dis_from_goal(double y);
extern double ticks_to_inches(double ticks);
extern Position odom(Position frame, double left_ticks, double right_ticks);
extern int Latency_Comp();

#endif