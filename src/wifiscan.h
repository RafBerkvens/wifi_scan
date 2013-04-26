/*
    The WifiScan class allows WiFi scans using iwlib.h.
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


#ifndef WIFISCAN_H
#define WIFISCAN_H

#include <cstdio>
#include <string>
#include <map>

#include <iwlib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <wifi_scan/Fingerprint.h>

#define WIFISCAN_ERROR_OPENING_IOCTL_SOCKET -1
#define WIFISCAN_ERROR_IN_IW_SCAN -2

/**
 * @brief The wifi_scan main class.
 *
 * The WifiScan class allows for easy operations on the iwlib library, and easy
 * publishing to the defined topics.
 **/
class WifiScan
{
  std::string interface_;

public:
  /**
   * @name 01. Constructors and destructor
   **/
  ///@{
  /**
   * @brief Default constructor.
   *
   * @param interface The Ethernet interface to scan. Defaults to "wlan0".
   **/
  WifiScan(std::string interface = "wlan0");
  /**
   * @brief Destructor
   *
   **/
  virtual ~WifiScan();
  ///@}

  /**
   * @name 02. Configuration
   **/
  ///@{
  /**
   * @brief Get the interface on which scans are performed.
   *
   * @return The interface as std::string.
   **/
  std::string interface() const { return interface_; }
  /**
   * @brief Set the interface to perform scans on.
   *
   * @param interface An std::string that corresponds to a live Ethernet
   * interface.
   * @return void
   **/
  void set_interface(std::string interface) { this->interface_ = interface; }
  ///@}

  /**
   * @name 03. ROS operations
   **/
  ///@{
  /**
   * @brief Publishes a Fingerprint message to the Publisher pub.
   *
   * A fingerprint is a list of all visible WiFi access points' device addresses
   * linked with their corresponding received signal strength indication.
   *
   * @param pub A ROS Publisher, publishing wifi_scan::Fingerprint messages.
   * Example: ros::Publisher pub = node.advertise<wifi_scan::Fingerprint>
   * ("wifi_fp", 10);
   *
   * @return void
   *
   * @throw WIFISCAN_ERROR_OPENING_IOCTL_SOCKET Error opening ioctl socket.
   * @throw WIFISCAN_ERROR_IN_IW_SCAN Error in iw_scan().
   **/
  void createFingerprint(ros::Publisher *pub);
  ///@}
};

#endif // WIFISCAN_H
// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
