from traceback import print_tb
#!/usr/bin/env python
import roslaunch
import rospy
import time
import os, sys, stat
os.chmod("/dev/ttyUSB0", stat.S_IRWXU | stat.S_IRWXG|stat.S_IRWXO)
time.sleep(1);
rospy.loginfo("started")
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
tracking_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/yuxu/mower_ws/src/mower/src/open_mower/launch/mower.launch"])
tracking_launch.start()


rospy.loginfo("started")
while not rospy.is_shutdown():
    time.sleep(1)

tracking_launch.shutdown()
