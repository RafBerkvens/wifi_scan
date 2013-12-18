/*
 fingerprint_rviz vizualizes the fingerprints.
 Copyright (C) 2013  Rafael Berkvens rafael.berkvens@ua.ac.be

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>
#include <map>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <wifi_scan/Fingerprint.h>

namespace jet
{
double interpolate(double val, double y0, double x0, double y1, double x1)
{
  return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

double base(double val)
{
  if (val <= -0.75)
    return 0;
  else if (val <= -0.25)
    return interpolate(val, 0.0, -0.75, 1.0, -0.25);
  else if (val <= 0.25)
    return 1.0;
  else if (val <= 0.75)
    return interpolate(val, 1.0, 0.25, 0.0, 0.75);
  else
    return 0.0;
}

double red(double gray)
{
  return base(gray - 0.5);
}
double green(double gray)
{
  return base(gray);
}
double blue(double gray)
{
  return base(gray + 0.5);
}
}

/**
 * @brief This is in fact the callback for when fingerprint messages are
 * available.
 **/
class FingerprintRviz
{
//   ros::NodeHandle *node_;
  ros::Publisher *pub_rviz_msgs_;
  //   ros::Subscriber sub_fingerprint_;

  tf::TransformBroadcaster odom_br_;
  tf::Transform odom_tr_;

  std::map<std::string, double> fingerprint_;

  visualization_msgs::Marker marker_;

  std::stringstream hexstream_;

  int index_;

  std::map<std::string, int> device_addresses_;
  int n_devices_;

public:
  FingerprintRviz(ros::Publisher *pub);
  virtual ~FingerprintRviz();

  void fingerprint_rvizCallback(
                                const wifi_scan::Fingerprint &fingerprint);
  /**
   * WARNING: depricated
   * @param odom
   */
  void odom_tfCallback(
                       const nav_msgs::Odometry &odom);
};

FingerprintRviz::FingerprintRviz(ros::Publisher *pub)
{
//   node_ = node;

  pub_rviz_msgs_ = pub;
//   sub_fingerprint_ = node_->subscribe(
//                        topic_name, 1000,
//                        fingerprint_rvizCallback);

  marker_.header.frame_id = "/base_link";
  marker_.header.stamp = ros::Time();
  marker_.ns = "fingerprint_rviz";
  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = 0;
  marker_.pose.position.y = 0;
  marker_.pose.position.z = 0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 0.0;
  marker_.scale.x = 0.1;
  marker_.scale.y = 0.1;
  marker_.scale.z = 0.1;
  marker_.color.a = 1.0;
  marker_.color.r = 0.0;
  marker_.color.g = 0.0;
  marker_.color.b = 0.0;

  hexstream_ << std::hex;

  index_ = 0;
  n_devices_ = 0;
}

FingerprintRviz::~FingerprintRviz()
{
  delete pub_rviz_msgs_;
}

void FingerprintRviz::fingerprint_rvizCallback(
    const wifi_scan::Fingerprint &fingerprint)
{
  fingerprint_.clear();
  if (fingerprint.list.empty())
  {
    ROS_WARN_STREAM("Empty list!");
  }
  wifi_scan::AddressRSSI address_rssi;
  for (int i = 0; i < fingerprint.list.size(); i++)
  {
    address_rssi = fingerprint.list[i];

    /* Put address and RSSI in fingerprint if address is unique*/
    std::pair<std::map<std::string, double>::iterator, bool> ret;
    ret = fingerprint_.insert(
        std::pair<std::string, double>(
                                       std::string(address_rssi.address),
                                       address_rssi.rssi));
    if (ret.second == false)
    {
      ROS_WARN_STREAM("Address not unique.");
    }
  }

  std::map<std::string, double>::iterator access_point;
  int i;
  for (access_point = fingerprint_.begin();
      access_point != fingerprint_.end();
      access_point++, index_++)
  {
    ROS_DEBUG_STREAM("device mac address: " << access_point->first);
    std::pair<std::map<std::string, int>::iterator, bool> check;
    check = device_addresses_.insert(
        std::pair<std::string, int>(access_point->first, n_devices_));
    if (check.second)
    {
      n_devices_++;
      ROS_DEBUG_STREAM("n_device_: " << n_devices_);
    }

    marker_.id = index_;
    //marker_.pose.position.z = (access_point->second + 100) / 10;
    marker_.pose.position.z =
        device_addresses_.find(access_point->first)->second / 10.0;
    ROS_DEBUG_STREAM("z position: " << marker_.pose.position.z);

    unsigned int colorr, colorg, colorb;
    // values: 30 since -30 highest seen value; 100.0 since values between
    // 0 and -100 theoretically; 0.6 since -90 (+30/100) lowest seen value
    colorr = jet::red(((access_point->second + 80) / 100.0)) * 255;
    colorg = jet::green(((access_point->second + 80) / 100.0)) * 255;
    colorb = jet::blue(((access_point->second + 80) / 100.0)) * 255;
//    hexstream_ << access_point->first[2] << access_point->first[3];
//    hexstream_ >> colorr;
//    hexstream_.clear(); // clear error flags
//    hexstream_ << access_point->first[4] << access_point->first[5];
//    hexstream_ >> colorg;
//    hexstream_.clear(); // clear error flags
//    hexstream_ << access_point->first[6] << access_point->first[7];
//    hexstream_ >> colorb;
//    hexstream_.clear(); // clear error flags
    ROS_DEBUG_STREAM("colors for " << access_point->second << ": "
                     << colorr << " " << colorg << " " << colorb);
    marker_.color.r = static_cast<int>(colorr) / 255.0;
    marker_.color.g = static_cast<int>(colorg) / 255.0;
    marker_.color.b = static_cast<int>(colorb) / 255.0;

    pub_rviz_msgs_->publish(marker_);
  }
}

void FingerprintRviz::odom_tfCallback(const nav_msgs::Odometry &odom)
{
  odom_tr_.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                 odom.pose.pose.position.y,
                                 odom.pose.pose.position.z));
  odom_tr_.setRotation(tf::Quaternion(odom.pose.pose.orientation.x,
                                      odom.pose.pose.orientation.y,
                                      odom.pose.pose.orientation.z,
                                      odom.pose.pose.orientation.w));
  odom_br_.sendTransform(tf::StampedTransform(odom_tr_, ros::Time::now(),
                                              "/odom",
                                              "/base_link"));
}

/**
 * @brief Create a ROS node that subscribes to the /wifi_fp topic and
 * vizualizes this information.
 *
 * This is the main function to set up the ROS node.
 **/
int main(int argc, char **argv)
{
  /* Set up ROS, get handle, set desired rate. */
  ros::init(argc, argv, "fingerprint_rviz");
  ros::NodeHandle node;
  ros::Rate rate(1);

  /* Get parameters from command line. */
  ros::NodeHandle private_node_handle_("~");
  std::string topic_name;
  private_node_handle_.param<std::string>("topic", topic_name, "wifi_fp");
  std::string pub_topic_name = topic_name + "_rviz";
  ros::Publisher pub_rviz_msgs = node.advertise<visualization_msgs::Marker>(
      pub_topic_name, 0);
  FingerprintRviz fingerprint_rviz(&pub_rviz_msgs);
  ros::Subscriber sub_fingerprint = node.subscribe(
      topic_name, 1000,
      &FingerprintRviz::fingerprint_rvizCallback,
      &fingerprint_rviz);
//  ros::Subscriber sub_odom = node.subscribe("/p3dx/odom", 1000,
//                                            &FingerprintRviz::odom_tfCallback,
//                                            &fingerprint_rviz);

  ros::spin();
}
// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
