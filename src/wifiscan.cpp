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

struct wireless_scan * WifiScan::iw_process_scanning_token(
    struct iw_event * event, struct wireless_scan * wscan)
{
  struct wireless_scan * oldwscan;

  /* Now, let's decode the event */
  switch (event->cmd)
  {
    case SIOCGIWAP:
      /* New cell description. Allocate new cell descriptor, zero it. */
      oldwscan = wscan;
      wscan = (struct wireless_scan *)malloc(sizeof(struct wireless_scan));
      if (wscan == NULL)
        return (wscan);
      /* Link at the end of the list */
      if (oldwscan != NULL)
        oldwscan->next = wscan;

      /* Reset it */
      bzero(wscan, sizeof(struct wireless_scan));

      /* Save cell identifier */
      wscan->has_ap_addr = 1;
      memcpy(&(wscan->ap_addr), &(event->u.ap_addr), sizeof(sockaddr));
      break;
    case SIOCGIWNWID:
      wscan->b.has_nwid = 1;
      memcpy(&(wscan->b.nwid), &(event->u.nwid), sizeof(iwparam));
      break;
    case SIOCGIWFREQ:
      wscan->b.has_freq = 1;
      wscan->b.freq = iw_freq2float(&(event->u.freq));
      wscan->b.freq_flags = event->u.freq.flags;
      break;
    case SIOCGIWMODE:
      wscan->b.mode = event->u.mode;
      if ((wscan->b.mode < IW_NUM_OPER_MODE) && (wscan->b.mode >= 0))
        wscan->b.has_mode = 1;
      break;
    case SIOCGIWESSID:
      wscan->b.has_essid = 1;
      wscan->b.essid_on = event->u.data.flags;
      memset(wscan->b.essid, '\0', IW_ESSID_MAX_SIZE + 1);
      if ((event->u.essid.pointer) && (event->u.essid.length))
        memcpy(wscan->b.essid, event->u.essid.pointer, event->u.essid.length);
      break;
    case SIOCGIWENCODE:
      wscan->b.has_key = 1;
      wscan->b.key_size = event->u.data.length;
      wscan->b.key_flags = event->u.data.flags;
      if (event->u.data.pointer)
        memcpy(wscan->b.key, event->u.essid.pointer, event->u.data.length);
      else
        wscan->b.key_flags |= IW_ENCODE_NOKEY;
      break;
    case IWEVQUAL:
      /* We don't get complete stats, only qual */
      wscan->has_stats = 1;
      memcpy(&wscan->stats.qual, &event->u.qual, sizeof(struct iw_quality));
      break;
    case SIOCGIWRATE:
      /* Scan may return a list of bitrates. As we have space for only
       * a single bitrate, we only keep the largest one. */
      if ((!wscan->has_maxbitrate) ||
          (event->u.bitrate.value > wscan->maxbitrate.value))
      {
        wscan->has_maxbitrate = 1;
        memcpy(&(wscan->maxbitrate), &(event->u.bitrate), sizeof(iwparam));
      }
    case IWEVCUSTOM:
      /* How can we deal with those sanely ? Jean II */
    default:
      break;
  } /* switch(event->cmd) */

  return (wscan);
}

int WifiScan::scan_channels(wireless_scan_head * context)
{
  struct iwreq wrq;
  struct iw_scan_req scanopt; // Options for 'set'.
  int scanflags = 0; // Flags for scan.
  unsigned char * buffer = NULL; // Results.
  int buflen = IW_SCAN_MAX_DATA; // Min for compat WE < 17.
  struct iw_range range;
  int has_range;
  struct timeval tv; // Select timeout.
  int timeout = 15000000; // 15 s.

  /* Get and check range stuff.
   * I would like to update this function to a class method, so that this range
   * info can be a private variable and must not be constantly checked.
   * Also, the function should be void and use throws instead of returns to
   * indicate errors.
   */
  has_range = (iw_get_range_info(socket_, interface_, &range) >= 0);
  if ((!has_range) || (range.we_version_compiled < 14))
  {
    std::cerr << interface_ << "does not support scanning." << std::endl;
    return SCAN_CHANNELS_NO_SCAN_SUPPORT;
  }

  /* Initialize timeout value -> 250 ms between set and first get.
   * This can potentially be increased in speed.
   */
  tv.tv_sec = 0;
  tv.tv_usec = 250000;

  /* Clean up set arguments. */
  memset(&scanopt, 0, sizeof(scanopt));

  /* Set options. */
  double channel_to_freq;
  iw_freq freq;
  for (int i = 0; i < channels_.size(); i++)
  {
    iw_channel_to_freq(channels_[i], &channel_to_freq, &range);
    iw_float2freq(channel_to_freq, &freq);
    scanopt.channel_list[i] = freq;
  }
  scanopt.num_channels = channels_.size();

  wrq.u.data.pointer = (caddr_t)&scanopt;
  wrq.u.data.length = sizeof(scanopt);
  wrq.u.data.flags = scanflags;

  /* Initiate scanning. */
  if (iw_set_ext(socket_, interface_, SIOCSIWSCAN, &wrq) < 0)
  {
    if ((errno != EPERM) || (scanflags != 0))
    {
      std::cerr << interface_ << "does not support scanning: "
          << strerror(errno) << std::endl;
      return SCAN_CHANNELS_NO_SCAN_SUPPORT;
    }
    tv.tv_usec = 0;
  }
  usleep(tv.tv_usec);
  timeout -= tv.tv_usec;

  while (true)
  {
    fd_set rfds;
    int last_fd;
    int ret;

    FD_ZERO(&rfds);
    last_fd = -1;

    ret = select(last_fd + 1, &rfds, NULL, NULL, &tv);
    if (ret < 0)
    {
      if (errno == EAGAIN || errno == EINTR)
        continue;
      std::cerr << "Unhandled signal, exiting..." << std::endl;
      return SCAN_CHANNELS_UNHANDLED_SIGNAL;
    }

    if (ret == 0)
    {
      unsigned char * newbuf;

      realloc:
      /* Allocate or reallocate the buffer. */
      newbuf = (unsigned char *)realloc(buffer, buflen);
      if (newbuf == NULL)
      {
        if (buffer)
          free(buffer);
        std::cerr << "Allocation failed." << std::endl;
        return SCAN_CHANNELS_ALLOCATION_FAILED;
      }
      buffer = newbuf;

      /* Try to read the results. */
      wrq.u.data.pointer = buffer;
      wrq.u.data.flags = 0;
      wrq.u.data.length = buflen;
      if (iw_get_ext(socket_, interface_, SIOCGIWSCAN, &wrq) < 0)
      {
        /* Check if buffer was too small (WE-17 only) */
        if ((errno == E2BIG) && (range.we_version_compiled > 16)
            && (buflen < 0xFFFF))
        {
          /* Some driver may return very large scan results, either
           * because there are many cells, or because they have many
           * large elements in cells (like IWEVCUSTOM). Most will
           * only need the regular sized buffer. We now use a dynamic
           * allocation of the buffer to satisfy everybody. Of course,
           * as we don't know in advance the size of the array, we try
           * various increasing sizes. Jean II */

          /* Check if the driver gave us any hints. */
          if (wrq.u.data.length > buflen)
            buflen = wrq.u.data.length;
          else
            buflen *= 2;

          /* wrq.u.data.length is 16 bits so max size is 65535 */
          if (buflen > 0xFFFF)
            buflen = 0xFFFF;

          /* Try again */
          goto realloc;
        }

        /* Check if results not available yet */
        if (errno == EAGAIN)
        {
          /* Restart timer for only 100ms*/
          tv.tv_sec = 0;
          tv.tv_usec = 100000;
          timeout -= tv.tv_usec;
          if (timeout > 0)
          {
            usleep(tv.tv_usec);
            continue; /* Try again later */
          }
        }

        /* Bad error. */
        free(buffer);
        std::cerr << interface_ << ": failed to read scan data: "
            << strerror(errno) << std::endl;
        return SCAN_CHANNELS_FAILED_TO_READ;
      }
      else
        break;
    }
  }

  if (wrq.u.data.length)
  {
    struct iw_event iwe;
    struct stream_descr stream;
    struct wireless_scan * wscan = NULL;
    struct iwscan_state state = { /*.ap_num =*/1, /*.val_index =*/0};
    int ret;

    iw_init_event_stream(&stream, (char *)buffer, wrq.u.data.length);
    /* This is dangerous, we may leak user data... */
    context->result = NULL;

    /* Look every token */
    do
    {
      /* Extract an event and print it. */
      ret = iw_extract_event_stream(&stream, &iwe, range.we_version_compiled);
      if (ret > 0)
      {
        /* Convert to wireless_scan struct */
        wscan = iw_process_scanning_token(&iwe, wscan);
        /* Check problems. */
        if (wscan == NULL)
        {
          free(buffer);
          errno = ENOMEM;
          return SCAN_CHANNELS_PROBLEMS_PROCESSING;
        }
        /* Save head of list */
        if (context->result == NULL)
          context->result = wscan;
      }
    } while (ret > 0);

    std::cout << interface_ << ": scan completed" << std::endl;
  }
  else
    std::cout << interface_ << ": no scan results" << std::endl;

  free(buffer);
  return 0;
}

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

WifiScan::WifiScan(std::vector<int> channels, std::string interface)
{
  interface_ = new char[interface.length() + 1];
  std::strcpy(interface_, interface.c_str());
  channels_ = channels;

  if ((socket_ = iw_sockets_open()) < 0)
    throw WIFISCAN_ERROR_OPENING_IOCTL_SOCKET;
  kernel_version_ = iw_get_kernel_we_version();
  if (iw_get_range_info(socket_, interface_, &range_) < 0)
    throw WIFISCAN_ERROR_GETTING_RANGE_INFO;
}

WifiScan::~WifiScan()
{
  iw_sockets_close(socket_);
}

void WifiScan::createFingerprint(ros::Publisher *pub)
{
  wireless_scan_head scan_context;
  ROS_DEBUG_STREAM("uid: " << geteuid());
  if (scan_channels(&scan_context) < 0)
    throw WIFISCAN_ERROR_IN_IW_SCAN;
  if (scan_context.result != 0)
    ROS_DEBUG_STREAM("Fingerprint collected, publishing...");

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
