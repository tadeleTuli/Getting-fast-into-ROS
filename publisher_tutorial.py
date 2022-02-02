import rospy
from geometry_msgs.msg import Twist
import sys

def moveFunction_turtle(linear_velocity, angular_velocity):
    """Move turtle robot with input velocity"""
    rospy.init_node('moveFunction_turtle', anonymous=True)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    velocity = Twist()
    while not rospy.is_shutdown():
	velocity.linear.x = linear_velocity
	velocity.angular.z = angular_velocity

        rospy.loginfo("Linear velocity = %f: Angular velocity = %f: ", linear_velocity, angular_velocity)
        pub.publish(velocity)
        rate.sleep()


if __name__ == '__main__':
    try:
        moveFunction_turtle(float(sys.argv[1]), float(sys.argv[2]))
    except rospy.ROSInterruptException:
        pass
