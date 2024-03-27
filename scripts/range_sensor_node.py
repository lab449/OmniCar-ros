#!/usr/bin/env python
# license removed for brevity
import rospy
from range_sensor_driver import RangeSensorDriver

SHUTDOWN_PINS = []

def main():
    sensors = RangeSensorDriver(SHUTDOWN_PINS)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    rospy.init_node('range_sensor_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
