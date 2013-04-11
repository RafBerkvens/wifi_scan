/*
    fingerprint publishes the RSSI of all visible WiFi AP's to wifi_fp.
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

#include <cstdio>
#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <iwlib.h>

#include <wifi_scan/Fingerprint.h>

#include "wifiscan.h"

/**
 * @brief Create a ROS node that publishes fingerprints.
 *
 * This is the main function to set up the ROS node.
 **/
int main(int argc, char **argv)
{
  /* Set up ROS, get handle, create publisher, set desired rate. */
  ros::init(argc, argv, "fingerprint");
  ros::NodeHandle node;
  ros::Publisher pub_fingerprint = node.advertise<wifi_scan::Fingerprint>
                                   ("wifi_fp", 10);
  ros::Rate rate(1);

  /* Create scanning object and start fingerprinting. */
  WifiScan wifiscan;
  while(node.ok())
  {
    wifiscan.createFingerprint(&pub_fingerprint);
    ros::spinOnce();
    rate.sleep();
  }
}

// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
