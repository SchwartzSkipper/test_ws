#ifndef BZ_PLANNER_UTILITY_H
#define BZ_PLANNER_UTILITY_H
#include <vector>
#include <iostream>
#include <string>
#include <geometry_msgs/Pose2D>

typedef struct Bzstruct_
{
    geometry_msgs::Pose2D pr, ps, pt, pu, pv;
    double ps_tan, pt_tan, pr_len, px_len, px_offset, pr_dir, ps_offset, pt_offset;
} Bzstruct;

#endif