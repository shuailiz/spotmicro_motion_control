#!/home/shuai/.pyenv/shims/python
import rospy
from spotmicro_teleop.teleop import Teleop


if __name__ == '__main__':
    rospy.init_node("spot_teleop_node")
    rospy.loginfo("Start teleop node")
    teleop = Teleop()

    teleop.run()
