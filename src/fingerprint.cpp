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

#include <wifi_scan/RSSI.h>

/** Main function to set up ROS node.
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "fingerprint");
  ros::NodeHandle node;

  ros::Publisher pub_fingerprint = node.advertise<wifi_scan::RSSI>
                                   ("wifi_fp", 10);
  ros::Rate rate(1);

  while(node.ok())
  {
    int sockfd;
    wireless_scan_head scan_context;

    if((sockfd = iw_sockets_open()) < 0)
    {
      ROS_ERROR("Error opening ioctl socket");
      return 0;
    }
    int we_kernel_version = iw_get_kernel_we_version();

    if(iw_scan(sockfd, "wlan0", we_kernel_version, &scan_context) < 0)
    {
      ROS_ERROR("Error in iw_scan()");
      return 0;
    }

    iw_sockets_close(sockfd);

    for(wireless_scan *i = scan_context.result; i != 0; i = i->next)
    {
      double dBm;
      if(i->stats.qual.updated & IW_QUAL_DBM)
      {
        dBm = i->stats.qual.level;
        if(i->stats.qual.level >= 64)
          dBm -= 0x100;
      }
      else if(i->stats.qual.updated & IW_QUAL_RCPI)
      {
        dBm = (i->stats.qual.level / 2.0) - 110.0;
      }

      char address[128];
      snprintf(address, 128, "%02x:%02x:%02x:%02x:%02x:%02x"
               , (unsigned char)i->ap_addr.sa_data[0]
               , (unsigned char)i->ap_addr.sa_data[1]
               , (unsigned char)i->ap_addr.sa_data[2]
               , (unsigned char)i->ap_addr.sa_data[3]
               , (unsigned char)i->ap_addr.sa_data[4]
               , (unsigned char)i->ap_addr.sa_data[5]);
      char strength[32];
      snprintf(strength, 32, "%g", dBm);

      wifi_scan::RSSI rssi;
      rssi.header.stamp = ros::Time::now();
      rssi.address = address;
      rssi.rssi = atoi(strength);
      pub_fingerprint.publish(rssi);
    }
    ros::spinOnce();
    rate.sleep();
  }
}

// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
