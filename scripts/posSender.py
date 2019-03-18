#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

POS_REF = Vector3()

def chatter_posref():
    pub = rospy.Publisher('/cmd/pos_ref', Vector3, queue_size=10)
    rospy.init_node('chatter_posref', anonymous=True)
    rate = rospy.Rate(20) # 20hz

    count=0

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        if count<600:
            POS_REF.x = 0.7
            POS_REF.y = 0
            POS_REF.z = 2
        elif count<1200:
            POS_REF.x = 0.7 * math.cos(0.2 * (count - 650) / 20)
            POS_REF.y = 0.7 * math.sin(0.2 * (count - 650) / 20)
            POS_REF.z = 2
        else:
            POS_REF.x = 0.7 * math.cos(0.2 * (1200 - 650) / 20)
            POS_REF.y = 0.7 * math.sin(0.2 * (1200 - 650) / 20)
            POS_REF.z = 2

        count=count+1
        if count>1000:
            count=0

        # POS_REF.x = 0
        # POS_REF.y = 0
        # POS_REF.z = 0.9

        rospy.loginfo(POS_REF)
        pub.publish(POS_REF)
        rate.sleep()

if __name__ == '__main__':
    try:
        chatter_posref()
    except rospy.ROSInterruptException:
        pass
