#!/usr/bin/env python

import rospy

from scapy.all import *

from wifi_scan.msg import rssi

def packet_handler(pkt):
  if pkt.haslayer(Dot11):
    if pkt.type == 0 and pkt.subtype == 8 :
      print "%s" %pkt.addr2
      return (pkt.addr2, -(256-ord(pkt.notdecoded[-4:-3])))

def wifi_scan():
  pub = rospy.Publisher('rssi', rssi)
  rospy.init_node('wifi_scan')
  while not rospy.is_shutdown():
    packet_list = sniff(iface = "mon0", count = 1) # contains only one element
    (address, rssi_v) = packet_handler(packet_list[0])

if __name__ == '__main__':
  try:
    wifi_scan()
  except rospy.ROSInterruptException:
    pass
