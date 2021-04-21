#ifndef POLYGON_UTILS_H
#define POLYGON_UTILS_H

#include <string>

#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>

namespace laser_filters
{
inline double sign0(double x);
void padPolygon(geometry_msgs::Polygon& polygon, double padding);
double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
geometry_msgs::Polygon makePolygonFromXMLRPC(const XmlRpc::XmlRpcValue& polygon_xmlrpc,
                                             const std::string& full_param_name);
std::vector<std::vector<float> > parseVVF(const std::string& input, std::string& error_return);
geometry_msgs::Polygon makePolygonFromString(const std::string& polygon_string,
                                             const geometry_msgs::Polygon& last_polygon);
std::string polygonToString(geometry_msgs::Polygon polygon);
}  // namespace laser_filters

#endif  // POLYGON_UTILS_H
