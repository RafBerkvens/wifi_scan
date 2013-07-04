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

#include "wifiscan.h"

bool WifiScan::DeviceAddressCompare::operator ()(const std::string &lhs,
                                                 const std::string &rhs) const
                                                 {
  return lhs.substr(0, 10) < rhs.substr(0, 10);
}

double WifiScan::MeanRSSI(const std::vector<double> &RSSIValues)
{
  double total = 0.0;
  for (std::vector<double>::const_iterator it = RSSIValues.begin();
      it != RSSIValues.end(); it++)
  {
    total += *it;
  }
  return total / RSSIValues.size();
}

WifiScan::WifiScan(std::string interface)
{
  this->interface_ = interface;
}

WifiScan::~WifiScan()
{

}

void WifiScan::createFingerprint(ros::Publisher *pub)
{
  /* Create socket, perform scan, close socket. */
  int sockfd; /* socket to the kernel */
  if ((sockfd = iw_sockets_open()) < 0)
    throw WIFISCAN_ERROR_OPENING_IOCTL_SOCKET;
  wireless_scan_head scan_context;
  int kernel_version = iw_get_kernel_we_version();
  char *interface = new char[interface_.length() + 1];
  std::strcpy(interface, interface_.c_str());
  ROS_DEBUG_STREAM("uid: " << geteuid());
  if (iw_scan(sockfd, interface, kernel_version, &scan_context) < 0)
    throw WIFISCAN_ERROR_IN_IW_SCAN;
  if (scan_context.result != 0)
    ROS_DEBUG_STREAM("Fingerprint collected, publishing...");
  iw_sockets_close(sockfd);

  /* Loop over result, build fingerprint. */
  std::map<std::string, double, DeviceAddressCompare> fingerprint;
  std::map<std::string, std::vector<double> > devicesWithMoreSSIDs;
  for (wireless_scan *i = scan_context.result; i != 0; i = i->next)
  {
    /* Retrieve device address. */
    char address[128];
    snprintf(address, 128, "%02X%02X%02X%02X%02X%02X",
             (unsigned char)i->ap_addr.sa_data[0],
             (unsigned char)i->ap_addr.sa_data[1],
             (unsigned char)i->ap_addr.sa_data[2],
             (unsigned char)i->ap_addr.sa_data[3],
             (unsigned char)i->ap_addr.sa_data[4],
             (unsigned char)i->ap_addr.sa_data[5]);

    /* Retrieve RSSI */
    double dBm;
    if (i->stats.qual.updated & IW_QUAL_DBM)
    {
      dBm = i->stats.qual.level;
      if (i->stats.qual.level >= 64)
        dBm -= 0x100;
    }
    else if (i->stats.qual.updated & IW_QUAL_RCPI)
    {
      dBm = (i->stats.qual.level / 2.0) - 110.0;
    }

    /* Put address and RSSI in fingerprint if address is unique*/
    std::pair<std::map<std::string, double>::iterator, bool> ret;
    ret = fingerprint.insert(
        std::pair<std::string, double>(std::string(address), dBm));
    if (ret.second == false)
    {
      std::vector<double> RSSIValues;
      std::string newSSID = std::string(address);
      std::map<std::string, std::vector<double> >::iterator deviceWithMoreSSIDs;
      deviceWithMoreSSIDs = devicesWithMoreSSIDs.find(newSSID.substr(0, 10));

      if (deviceWithMoreSSIDs == devicesWithMoreSSIDs.end())
      {
        if (ret.first->first.substr(11, 2) == newSSID.substr(11, 2))
        {
          ROS_WARN_STREAM("Address not unique.");
        }
        else
        {
          RSSIValues.push_back(dBm);
          RSSIValues.push_back(ret.first->second);
          newSSID = newSSID.substr(0, 10);
          devicesWithMoreSSIDs.insert(
              std::pair<std::string, std::vector<double> >(newSSID,
                                                           RSSIValues));
          dBm = MeanRSSI(RSSIValues);
          fingerprint.erase(ret.first);
          fingerprint.insert(
              std::pair<std::string, double>(newSSID + "**", dBm));
        }
      }
      else
      {
        RSSIValues = deviceWithMoreSSIDs->second;
        RSSIValues.push_back(dBm);
        deviceWithMoreSSIDs->second = RSSIValues;
        dBm = MeanRSSI(RSSIValues);
        ret.first->second = dBm;
      }
    }
  }

  /* Create fingerprint message, publish message. */
  wifi_scan::Fingerprint fingerprint_message;
  fingerprint_message.list.clear();
  for (std::map<std::string, double>::iterator it = fingerprint.begin();
      it != fingerprint.end();
      it++)
  {
    wifi_scan::AddressRSSI address_rssi;
    address_rssi.address = it->first;
    address_rssi.rssi = it->second;
    ROS_DEBUG_STREAM(it->first << ", " << it->second);
    fingerprint_message.list.push_back(address_rssi);
  }
  fingerprint_message.header.stamp = ros::Time::now();
  pub->publish(fingerprint_message);
}
// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
