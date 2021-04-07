#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>

#include "polygon_utils.h"

/** @brief Same as sign(x) but returns 0 if x is 0. */
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

void padPolygon(geometry_msgs::Polygon& polygon, double padding)
{
  // pad polygon in place
  for (unsigned int i = 0; i < polygon.points.size(); i++)
  {
    geometry_msgs::Point32& pt = polygon.points[i];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the polygon specification (param %s) must be numbers. Found value %s.",
              full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the polygon specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

geometry_msgs::Polygon makePolygonFromXMLRPC(const XmlRpc::XmlRpcValue& polygon_xmlrpc,
                                             const std::string& full_param_name)
{
  // Make sure we have an array of at least 3 elements.
  if (polygon_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      polygon_xmlrpc.size() > 0 && polygon_xmlrpc.size() < 3)
  {
    ROS_FATAL("The polygon (parameter %s) must be specified as nested list on the parameter server with at least "
              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]",
              full_param_name.c_str());

    throw std::runtime_error("The polygon must be specified as nested list on the parameter server with at least "
                             "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 pt;

  for (int i = 0; i < polygon_xmlrpc.size(); ++i)
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = polygon_xmlrpc[i];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2)
    {
      ROS_FATAL("The polygon (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                full_param_name.c_str());
      throw std::runtime_error("The polygon must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[0], full_param_name);
    pt.y = getNumberFromXMLRPC(point[1], full_param_name);

    polygon.points.push_back(pt);
  }
  return polygon;
}

std::vector<std::vector<float> > parseVVF(const std::string& input, std::string& error_return)
{  // Source: https://github.com/ros-planning/navigation/blob/melodic-devel/costmap_2d/src/array_parser.cpp
  std::vector<std::vector<float> > result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2)
        {
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0)
        {
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1)
        {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2)
        {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
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

geometry_msgs::Polygon makePolygonFromString(const std::string& polygon_string,
                                             const geometry_msgs::Polygon& last_polygon)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(polygon_string, error);

  if (error != "")
  {
    ROS_ERROR("Error parsing polygon parameter: '%s'", error.c_str());
    ROS_ERROR(" Polygon string was '%s'.", polygon_string.c_str());
    return last_polygon;
  }

  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 point;

  // convert vvf into points.
  if (vvf.size() < 3 && vvf.size() > 0)
  {
    ROS_WARN("You must specify at least three points for the robot polygon");
    return last_polygon;
  }

  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[i].size() == 2)
    {
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      point.z = 0;
      polygon.points.push_back(point);
    }
    else
    {
      ROS_ERROR("Points in the polygon specification must be pairs of numbers. Found a point with %d numbers.",
                int(vvf[i].size()));
      return last_polygon;
    }
  }

  return polygon;
}

std::string polygonToString(geometry_msgs::Polygon polygon)
{
  std::string polygon_string = "[";
  bool first = true;
  for (auto point : polygon.points)
  {
    if (!first)
    {
      polygon_string += ", ";
    }
    first = false;
    polygon_string += "[" + std::to_string(point.x) + ", " + std::to_string(point.y) + "]";
  }
  polygon_string += "]";
  return polygon_string;
}
