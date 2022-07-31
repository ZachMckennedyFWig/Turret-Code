#ifndef general_H
#define general_H

#include "../../Structures/structures.h"

extern void reset_encoders();
extern Encoder get_drive_encoder();
extern double ticks_to_inches(double ticks);
extern void print_coords(int x, int y);
extern void move_left_vol(double vol);
extern void move_right_vol(double vol);
extern void set_drive_coast();

#endif