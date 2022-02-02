import sys
import rospy
from geometry_msgs.msg import Twist
import datetime

def callback_pose(data):
    """Get the ROS message"""   
    print(data)


def listener():
   """Subscribe to ROS topic node"""

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback_pose)
    rospy.spin()

if __name__ == '__main__':
    listener()
