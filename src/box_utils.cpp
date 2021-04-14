#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <ros/ros.h>

#include "box_utils.h"

std::string boxToString(const Box& box)
{
  std::string box_string = "[";

  box_string +=
      "[" + std::to_string(box.min.x) + ", " + std::to_string(box.min.y) + ", " + std::to_string(box.min.z) + "]";

  box_string += ", ";

  box_string +=
      "[" + std::to_string(box.max.x) + ", " + std::to_string(box.max.y) + ", " + std::to_string(box.max.z) + "]";

  box_string += "]";
  return box_string;
}

Box makeBoxFromTwoPoints(const std::vector<geometry_msgs::Point32>& points)
{
  Box box;
  if (points.size() == 2)
  {
    box.min.x = std::min(points[0].x, points[1].x);
    box.min.y = std::min(points[0].y, points[1].y);
    box.min.z = std::min(points[0].z, points[1].z);

    box.max.x = std::max(points[0].x, points[1].x);
    box.max.y = std::max(points[0].y, points[1].y);
    box.max.z = std::max(points[0].z, points[1].z);
  }

  return box;
}

Box makeBoxFromXMLRPC(const XmlRpc::XmlRpcValue& box_xmlrpc, const std::string& full_param_name)
{
  // checks if an array has just two elements.
  if (box_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || box_xmlrpc.size() != 2)
  {
    ROS_FATAL("The box (parameter %s) must be specified as nested list on the parameter server with two points"
              "i.e. [[x1, y1, z1], [x2, y2, z2]]",
              full_param_name.c_str());

    throw std::runtime_error("The box must be specified as nested list on the parameter server with two points"
                             "i.e. [[x1, y1, z1], [x2, y2, z2]]");
  }

  std::vector<geometry_msgs::Point32> points;
  points.resize(box_xmlrpc.size());  // points.size() == 2

  for (int i = 0; i < box_xmlrpc.size(); ++i)
  {
    // checks if each element (point) has three elements,
    // i.e. x, y, and z coordinates
    XmlRpc::XmlRpcValue point_xmlrpc = box_xmlrpc[i];
    if (point_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || point_xmlrpc.size() != 3)
    {
      ROS_FATAL("The box (parameter %s) must be specified as list of lists,"
                "i.e. [[x1, y1, z1], [x2, y2, z2]], but this spec is not of that form.",
                full_param_name.c_str());
      throw std::runtime_error("The box must be specified as list of lists,"
                               "i.e. [[x1, y1, z1], [x2, y2, z2]], but this spec is not of that form");
    }

    points[i].x = getNumberFromXMLRPC(point_xmlrpc[0], full_param_name);
    points[i].y = getNumberFromXMLRPC(point_xmlrpc[1], full_param_name);
    points[i].z = getNumberFromXMLRPC(point_xmlrpc[2], full_param_name);
  }

  Box box = makeBoxFromTwoPoints(points);
  return box;
}

Box makeBoxFromString(const std::string& box_string, const Box& last_box)
{
  std::string error;
  std::vector<std::vector<float>> vvf = parseVVF(box_string, error);

  if (error != "")
  {
    ROS_ERROR("Error parsing box parameter: '%s'", error.c_str());
    ROS_ERROR(" Box string was '%s'.", box_string.c_str());
    return last_box;
  }

  // convert vvf into points.
  if (vvf.size() != 2)
  {
    ROS_WARN("You must specify just two points to define a box");
    return last_box;
  }

  std::vector<geometry_msgs::Point32> points;
  points.resize(vvf.size());  // points.size() == 2

  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[i].size() == 3)
    {
      points[i].x = vvf[i][0];
      points[i].y = vvf[i][1];
      points[i].z = vvf[i][2];
    }
    else
    {
      ROS_ERROR("The box must be specified as list of lists,"
                "i.e. [[x1, y1, z1], [x2, y2, z2]], but this spec is not of that form");
      return last_box;
    }
  }

  Box box = makeBoxFromTwoPoints(points);
  return box;
}

Box padBox(const Box& box, double padding)
{
  Box box_padded = box;

  // pads the box
  box_padded.min.x -= padding;
  box_padded.min.y -= padding;
  box_padded.min.z -= padding;

  box_padded.max.x += padding;
  box_padded.max.y += padding;
  box_padded.max.z += padding;

  return box_padded;
}
