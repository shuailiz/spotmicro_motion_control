#!/usr/bin/python
import rospy
from sensor_msgs.msg import JointState
from spotmicro_motor_control.constants import JOINT_MOTOR_MAP


class JointStateRepublisher(object):
    def __init__(self):
        self.converted_joint_state = JointState()
        self.converted_joint_state.name = [
            "motor_front_left_shoulder",
            "motor_front_left_leg",
            "foot_motor_front_left",
            "motor_front_right_shoulder",
            "motor_front_right_leg",
            "foot_motor_front_right",
            "motor_rear_left_shoulder",
            "motor_rear_left_leg",
            "foot_motor_rear_left",
            "motor_rear_right_shoulder",
            "motor_rear_right_leg",
            "foot_motor_rear_right"
        ]
        self.converted_joint_state.position = [0] * 12
        self.joint_state_publisher = rospy.Publisher("joint_states",
                                                     JointState,
                                                     queue_size=1)
        self.real_joint_state_subscriber = rospy.Subscriber("joint_states_real",
                                                            JointState,
                                                            self.real_joint_state_cb)

    def real_joint_state_cb(self, msg):
        self.converted_joint_state.position = [
            msg.position[0],
            -msg.position[1],
            -msg.position[2],
            msg.position[3],
            msg.position[4],
            msg.position[5],
            -msg.position[6],
            -msg.position[7],
            -msg.position[8],
            -msg.position[9],
            msg.position[10],
            msg.position[11]
        ]

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.converted_joint_state.header.stamp = rospy.Time.now()
            self.joint_state_publisher.publish(self.converted_joint_state)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("spotmicro_joint_state_republisher")
    joint_state_repub = JointStateRepublisher()
    joint_state_repub.run()
        
