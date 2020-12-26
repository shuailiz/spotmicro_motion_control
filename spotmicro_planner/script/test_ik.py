#!/usr/bin/python
import rospy
from sensor_msgs.msg import JointState
from spotmicro_planner.ik_solver import SpotMicroIkSolver

if __name__ == '__main__':
    rospy.init_node("spotmicro_test_ik")
    sp_ik_solver = SpotMicroIkSolver(l_shoulder=0.055,
                                     l_upper_arm=0.12,
                                     l_forearm=0.115,
                                     l_x=0.072,
                                     l_y=0.186,
                                     shoulder_lims=[-1.6, 1.6],
                                     elbow_lims=[-1.6, 1.6],
                                     wrist_lims=[-1.6, 2.5])
    pub = rospy.Publisher("joint_states_real", JointState, queue_size=1)
    
    rate = rospy.Rate(50)

    solution = sp_ik_solver.solve_ik(0, 0, -0.04, 0, 0, 0.3)

    msg = JointState()
    msg.name = ["motor_front_left_shoulder",
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
                "foot_motor_rear_right"]
    msg.position = solution["fl"].tolist() + solution["fr"].tolist() \
                   + solution["rl"].tolist() + solution["rr"].tolist()
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
