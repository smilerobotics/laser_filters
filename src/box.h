#ifndef BOX_H
#define BOX_H

#include <geometry_msgs/Point32.h>

struct Box
{
  // A box (cuboid) in without rotation can be described by two different points
  // on a Cartesian coordinate system, as shown in the following figure.
  //
  //               .---. max
  //              /   /|
  // z^  y       .---. .
  //  |/         |   |/
  //  .-> x  min .---.
  geometry_msgs::Point32 min;
  geometry_msgs::Point32 max;
};

#endif
