from traceback import print_tb
#!/usr/bin/env python
import roslaunch
import rospy
import time
import os, sys, stat
password = '789456123'
command = 'chmod 777 /dev/ttyUSB0'
str = os.system('echo %s | sudo -S %s' % (password,command))
command = 'chmod 777 /dev/ttyUSB1'
str = os.system('echo %s | sudo -S %s' % (password,command))
command = 'chmod 777 /dev/ttyUSB2'
str = os.system('echo %s | sudo -S %s' % (password,command))

#str = os.execl('/home/yuxu/mower_ws/build/mower/src/mower_location/neptune.location_main','neptune.location_main')
# os.chmod("/dev/ttyUSB0", stat.S_IRWXU | stat.S_IRWXG|stat.S_IRWXO)

time.sleep(1);
rospy.loginfo("started")
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
tracking_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/yuxu/mower_ws/src/mower/src/open_mower/launch/mower.launch"])
tracking_launch.start()
str = os.system('exec /home/yuxu/mower_ws/build/mower/src/mower_location/neptune.localtion_main &')

rospy.loginfo("started")
while not rospy.is_shutdown():
    time.sleep(1)

tracking_launch.shutdown()
