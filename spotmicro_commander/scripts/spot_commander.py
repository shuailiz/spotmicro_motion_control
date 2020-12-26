#!/home/shuai/.pyenv/shims/python
import rospy
from spotmicro_commander.commander import SpotCommander


if __name__ == '__main__':
    rospy.init_node("spot_commander_node")

    rospy.loginfo("Start commander node")
    commander = SpotCommander()
    commander.run()
