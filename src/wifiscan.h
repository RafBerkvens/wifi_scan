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
#include <string>
#include <vector>

#include <iwlib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <wifi_scan/Fingerprint.h>

#define WIFISCAN_ERROR_OPENING_IOCTL_SOCKET -1
#define WIFISCAN_ERROR_IN_IW_SCAN -2
#define WIFISCAN_ERROR_GETTING_RANGE_INFO -3
#define WIFISCAN_ERROR_GETTING_BASIC_INFO -4

#define SCAN_CHANNELS_NO_SCAN_SUPPORT -1
#define SCAN_CHANNELS_UNHANDLED_SIGNAL -2
#define SCAN_CHANNELS_ALLOCATION_FAILED -3
#define SCAN_CHANNELS_FAILED_TO_READ -4
#define SCAN_CHANNELS_PROBLEMS_PROCESSING -5

/**
 * @brief The wifi_scan main class.
 *
 * The WifiScan class allows for easy operations on the iwlib library, and easy
 * publishing to the defined topics.
 **/
class WifiScan
{
  char * interface_;
  std::vector<int> channels_;

  struct iw_range range_;
  int socket_;
  int kernel_version_;

  /**
   * @brief Scan state and meta-information, used to decode events.
   *
   * See iwlist.c.
   *
   * @param ap_num Access point number 1->N.
   * @param val_index Value in table 0->(N-1).
   */
  struct iwscan_state
  {
    int ap_num;
    int val_index;
  };

  /**
   * @brief Bit to name mapping.
   *
   * See iwlist.c. Might not be applicable in current application.
   *
   * @param mask Bit mask for the value.
   * @param name Human readable name for the vlaue.
   */
  struct iwmask_name
  {
    unsigned int mask;
    const char * name;
  };

  /**
   * @brief Process/store one element from the scanning results in wireless_scan.
   *
   * @todo More documentation.
   *
   * @param event
   * @param wscan
   * @return
   */
  struct wireless_scan * iw_process_scanning_token(
      struct iw_event * event, struct wireless_scan * wscan);

  /**
   * @brief Scan the channels indicated in channels.
   *
   * This code is based on the iwlist print_scanning_info function. See source
   * code of the libiw project (iwlist.c).
   *
   * I would like to update this function to a class method, so that this range
   * info can be a private variable and must not be constantly checked.
   * Also, the function should be void and use throws instead of returns to
   * indicate errors.
   *
   * @param context Scan results.
   * @return 0 for success; see defines for other options.
   */
  int scan_channels(wireless_scan_head * context);

  struct DeviceAddressCompare
  {
    bool operator()(const std::string &lhs, const std::string &rhs) const;
  };

  double MeanRSSI(const std::vector<double> &RSSIValues);

public:
  /**
   * @name 01. Constructors and destructor
   **/
  ///@{
  /**
   * @brief Default constructor.
   *
   * @todo Design better so that no channels must be passed when all channels
   * must be scanned.
   *
   * @param channels The channels to scan. Defaults to all channels.
   * @param interface The Ethernet interface to scan. Defaults to "wlan0".
   **/
  WifiScan(std::vector<int> channels, std::string interface = "wlan0");
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
  std::string interface() const
  {
    return (std::string) interface_;
  }
  /**
   * @brief Set the interface to perform scans on.
   *
   * @param interface An std::string that corresponds to a live Ethernet
   * interface.
   * @return void
   **/
  void set_interface(std::string interface)
  {
    delete interface_;
    this->interface_ = new char[interface.length() + 1];
    std::strcpy(interface_, interface.c_str());
  }
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
