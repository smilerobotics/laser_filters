/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, SmileRobotics, Japan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <ros/ros.h>

#include "box_utils.h"

namespace laser_filters
{
std::string boxToString(const Box& box)
{
  std::stringstream box_stringstream;
  box_stringstream << "["
                   << "[" << box.min.x << ", " << box.min.y << ", " << box.min.z << "]"
                   << ", "
                   << "[" << box.max.x << ", " << box.max.y << ", " << box.max.z << "]"
                   << "]";

  return box_stringstream.str();
}

Box makeBoxFromTwoPoints(const geometry_msgs::Point32& point0, const geometry_msgs::Point32& point1)
{
  Box box;

  box.min.x = std::min(point0.x, point1.x);
  box.min.y = std::min(point0.y, point1.y);
  box.min.z = std::min(point0.z, point1.z);

  box.max.x = std::max(point0.x, point1.x);
  box.max.y = std::max(point0.y, point1.y);
  box.max.z = std::max(point0.z, point1.z);

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

  std::vector<geometry_msgs::Point32> points(box_xmlrpc.size());  // size must be 2

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

  return makeBoxFromTwoPoints(points[0], points[1]);
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

  std::vector<geometry_msgs::Point32> points(vvf.size());  // size must be 2

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

  return makeBoxFromTwoPoints(points[0], points[1]);
}

geometry_msgs::Polygon makePolygonFromBox(const Box& box)
{
  geometry_msgs::Polygon box_msg;
  box_msg.points.resize(4);
  box_msg.points[0].x = box.max.x;
  box_msg.points[0].y = box.max.y;
  box_msg.points[1].x = box.min.x;
  box_msg.points[1].y = box.max.y;
  box_msg.points[2].x = box.min.x;
  box_msg.points[2].y = box.min.y;
  box_msg.points[3].x = box.max.x;
  box_msg.points[3].y = box.min.y;

  return box_msg;
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

std::vector<std::vector<std::vector<float>>> parseVVVF(const std::string& input, std::string& error_return)
{
  std::stringstream input_ss(input);
  int depth = 0;

  std::vector<float> current_vector;
  std::vector<std::vector<float>> current_vvf;
  std::vector<std::vector<std::vector<float>>> result;

  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth == 2)
        {
          current_vvf.clear();
        }
        else if (depth == 3)
        {
          current_vector.clear();
        }
        else if (depth > 3)
        {
          error_return = "Array depth greater than 3";
          return result;
        }
        input_ss.get();
        break;
      case ']':
        depth--;
        if (depth < 0)
        {
          error_return = "More close ] than open [";
          return result;
        }
        else if (depth == 1)
        {
          result.push_back(current_vvf);
        }
        else if (depth == 2)
        {
          current_vvf.push_back(current_vector);
        }
        input_ss.get();
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 3)
        {
          std::stringstream err_ss;
          err_ss << "Numbers at depth " << depth << " . Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }

        // reads a float value
        float value;
        input_ss >> value;
        if (!!input_ss)
        {
          current_vector.push_back(value);
        }
        break;
    }
  }

  if (depth != 0)
  {
    error_return = "Unterminated vector string.";
  }
  else
  {
    error_return = "";
  }

  return result;
}
}  // namespace laser_filters
