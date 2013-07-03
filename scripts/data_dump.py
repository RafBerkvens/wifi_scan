#!/usr/bin/env python

#  data_dump dumps wifi scans into a csv file.
#  Copyright (C) 2013  Rafael Berkvens rafael.berkvens@ua.ac.be
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from wifi_scan.msg import Fingerprint
from std_msgs.msg import Header

def fingerprintCallback(fingerprint, f):
  rospy.loginfo(rospy.get_name() + ": I heard something")
  f.write(str(fingerprint.header.stamp) + ',')
  for address_rssi in fingerprint.list:
    values = address_rssi.address
    values += ','
    values += str(address_rssi.rssi)
    values +=','
    f.write(values)
  f.write('\n')

def groundTruthCallback(data, f):
  rospy.loginfo(rospy.get_name() + ": I heard something")
  f.write(str(data.stamp) + ',' + data.frame_id)
  f.write('\n')

def data_dump():
  rospy.init_node('data_dump')
  f = open('dump.csv', 'w')
  f.write('')
  f.close()
  f = open('dump.csv', 'a')
  rospy.Subscriber("/wifi_fp", Fingerprint, fingerprintCallback, f)
  rospy.Subscriber("/gt_nfc", Header, groundTruthCallback, f)
  rospy.spin()

if __name__ == '__main__':
  data_dump()