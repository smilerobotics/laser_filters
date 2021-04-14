#ifndef BOX_UTILS_H
#define BOX_UTILS_h

#include <vector>

#include <geometry_msgs/Point32.h>

#include "box.h"
#include "polygon_utils.h"

std::string boxToString(const Box& box);
Box makeBoxFromTwoPoints(const std::vector<geometry_msgs::Point32>& points);
Box makeBoxFromXMLRPC(const XmlRpc::XmlRpcValue& box_xmlrpc, const std::string& full_param_name);
Box makeBoxFromString(const std::string& box_string, const Box& last_box);
Box padBox(const Box& polygon, double padding);

#endif
