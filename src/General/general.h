#ifndef general_H
#define general_H

#include "../Structures/structures.h"

void reset_encoders();
Encoder get_drive_encoder();
double ticks_to_inches(double ticks);
void print_coords(int x, int y);
void move_left_vol(double vol);
void move_right_vol(double vol);
void set_drive_coast();

#endif