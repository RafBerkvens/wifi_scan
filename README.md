# Install

Before building this project using catkin, the Wireless tools development files must be installed:

```
$ sudo apt-get install libiw-dev
```

Then, if the project is in `catkin/src/wifi_scan`, do in the `catkin` folder:

```
$ catkin_make
```

After building it, the executable is available in `catking/devel/lib/wifi_scan/`, named `fingerprint`. To create a good fingerprint, the interface must send an RSSI request packet to all visible routers and wait for the responses. To send such a packet, root privileges are necessary. Since ROS is usually not installed in the root environment, we use the s mode bit and set the user and group of the executable to root:

```
$ sudo chown root devel/lib/wifi_scan/fingerprint
$ sudo chgrp root devel/lib/wifi_scan/fingerprint
$ sudo chmod ug+s devel/lib/wifi_scan/fingerprint
```

To check if all went well try if you get the following or similar output:

```
$ ls -l devel/lib/wifi_scan/fingerprint
-rwsrwsr-x 1 root root 221926 jul 24 15:22 devel/lib/wifi_scan/fingerprint
```

# Usage

To start the node, do:

```
$ rosrun wifi_scan fingerprint
```

The default topic name is `/wifi_fp`, so you can change this when launching using:

```
$ rosrun wifi_scan fingerprint wifi_fp:=new_topic_name
```

The message type is a `wifi_scan/Fingerprint`. More information can be found in the Doxygen documentation. 

# Code

To browse the Doxygen documentation using Firefox as an example browser, do in the wifi_scan folder:

```
$ firefox doc/html/index.html
```
