#ifndef ROVER_CONTROL_H
#define ROVER_CONTROL_H

void rover_control_init(void);
void rover_control_set_cmd_vel(double linear_x, double angular_z);
void rover_control_update(int *left_mmps, int *right_mmps);

#endif