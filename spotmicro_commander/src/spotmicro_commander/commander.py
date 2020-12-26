import rospy
from spotmicro_planner.ik_solver import SpotMicroIkSolver
from spotmicro_planner.gait_generator import GaitGenerator
from spotmicro_teleop.msg import SpotMicroTeleop
from sensor_msgs.msg import JointState
from spotmicro_motor_control.constants import JOINT_MOTOR_MAP
from spotmicro_motor_control.constants import JOINT_SOFT_LIMITS
from spotmicro_motor_control.constants import ORDERED_LEG_NAME_ARRAY


class SpotCommander(object):
    L_SHOULDER = 0.055
    L_UPPER_ARM = 0.12
    L_FOREARM = 0.115
    L_X = 0.072
    L_Y = 0.186
    F_X = 0.12
    F_Y = 0.31
    F_Z = -0.21
    CONTROL_FREQ = 25

    def __init__(self):
        self.joint_control_pub = rospy.Publisher("spotmicro/joint_controller",
                                                 JointState,
                                                 queue_size=1
        )

        self.teleop_sub = rospy.Subscriber("spotmicro/teleop",
                                           SpotMicroTeleop,
                                           self.teleop_cb
        )

        shoulder_lims = dict()
        knee_lims = dict()
        wrist_lims = dict()

        shoulder_lims["FL"] = JOINT_SOFT_LIMITS["front_left_shoulder"]
        shoulder_lims["FR"] = JOINT_SOFT_LIMITS["front_right_shoulder"]
        shoulder_lims["RL"] = JOINT_SOFT_LIMITS["rear_left_shoulder"]
        shoulder_lims["RR"] = JOINT_SOFT_LIMITS["rear_right_shoulder"]

        knee_lims["FL"] = JOINT_SOFT_LIMITS["front_left_knee"]
        knee_lims["FR"] = JOINT_SOFT_LIMITS["front_right_knee"]
        knee_lims["RL"] = JOINT_SOFT_LIMITS["rear_left_knee"]
        knee_lims["RR"] = JOINT_SOFT_LIMITS["rear_right_knee"]

        wrist_lims["FL"] = JOINT_SOFT_LIMITS["front_left_wrist"]
        wrist_lims["FR"] = JOINT_SOFT_LIMITS["front_right_wrist"]
        wrist_lims["RL"] = JOINT_SOFT_LIMITS["rear_left_wrist"]
        wrist_lims["RR"] = JOINT_SOFT_LIMITS["rear_right_wrist"]
        
        self.ik_solver = SpotMicroIkSolver(l_shoulder=self.L_SHOULDER,
                                           l_upper_arm=self.L_UPPER_ARM,
                                           l_forearm=self.L_FOREARM,
                                           l_x=self.L_X,
                                           l_y=self.L_Y,
                                           f_x=self.F_X,
                                           f_y=self.F_Y,
                                           f_z=self.F_Z,
                                           shoulder_lims=shoulder_lims,
                                           knee_lims=knee_lims,
                                           wrist_lims=wrist_lims)

        self.gait_generator = GaitGenerator(
            L = 0.03,
            clearance_height = 0.025,
            num_bezier_points = 11,
            lateral_fraction = 0
        )

        self.in_place_roll = [-0.45, 0.45]
        self.in_place_pitch = [-0.5, 0.5]
        self.in_place_yaw = [-0.4, 0.4]
        self.in_place_z = [-0.06, 0.06]

        self.in_place = True
        self.walk = False
        self.walk_started = False
        
        self.in_place_roll_cmd = 0
        self.in_place_pitch_cmd = 0
        self.in_place_yaw_cmd = 0
        self.in_place_z_cmd = 0

        self.walk_max_x_speed = 0.03
        self.walk_max_y_speed = 0.03
        self.walk_max_yaw_rate = 0.5 * (1 / self.CONTROL_FREQ)

        self.walk_x = 0
        self.walk_y = 0
        self.walk_yaw = 0

        # TODO: use TF to do this!!!!!
        self.cur_T_bf = None  # Current foot coordinates relative to the body frame

    def teleop_cb(self, msg):
        if msg.in_place:
            self.in_place = True
            self.walk = False
            self.in_place_roll_cmd = msg.twist.angular.x
            self.in_place_pitch_cmd = msg.twist.angular.y
            self.in_place_yaw_cmd = msg.twist.angular.z
            self.in_place_z_cmd = msg.twist.linear.z
        else:
            self.in_place = False
            self.walk = True
            self.walk_x = msg.twist.linear.x
            self.walk_y = msg.twist.linear.y
            self.walk_yaw = msg.twist.angular.z

    def get_target(self, lower, upper, cmd):
        center = (lower + upper) / 2.0
        range = upper - center
        return center + range * cmd

    def run(self):
        rate = rospy.Rate(self.CONTROL_FREQ)
        while not rospy.is_shutdown():
            if self.in_place:
                self.walk_started = False
                cmd_msg = JointState()
                z_target = self.get_target(self.in_place_z[0],
                                           self.in_place_z[1],
                                           self.in_place_z_cmd
                )

                roll_target = self.get_target(self.in_place_roll[0],
                                              self.in_place_roll[1],
                                              self.in_place_roll_cmd
                )

                pitch_target = self.get_target(self.in_place_pitch[0],
                                               self.in_place_pitch[1],
                                               self.in_place_pitch_cmd
                )

                yaw_target = self.get_target(self.in_place_yaw[0],
                                             self.in_place_yaw[1],
                                             self.in_place_yaw_cmd
                )
                
                ik_solution, T_bf = self.ik_solver.solve_body_ik(
                    0,
                    0,
                    z_target,
                    roll_target,
                    pitch_target,
                    yaw_target
                )

                cmd_msg.name = JOINT_MOTOR_MAP.keys()
                if ik_solution["fl"] is None or \
                   ik_solution["fr"] is None or \
                   ik_solution["rl"] is None or \
                   ik_solution["rr"] is None:
                    continue
                cmd_msg.position = ik_solution["fl"].tolist() + \
                                   ik_solution["fr"].tolist() + \
                                   ik_solution["rl"].tolist() + \
                                   ik_solution["rr"].tolist()
                self.joint_control_pub.publish(cmd_msg)
                self.cur_T_bf = T_bf
            elif self.walk:
                # Revert the speed since x axis points backwards
                speed = -self.walk_max_x_speed * self.walk_x
                yaw_rate = self.walk_max_yaw_rate * self.walk_yaw
                lateral_fraction = 0
                if not self.walk_started:
                    gait_T_bf = self.gait_generator.generate_step_traj(
                        speed,
                        yaw_rate,
                        lateral_fraction,
                        self.cur_T_bf,
                        [1, 0, 0, 0]
                    )
                    self.walk_started = True
                else:
                    gait_T_bf = self.gait_generator.generate_step_traj(
                        speed,
                        yaw_rate,
                        lateral_fraction,
                        self.cur_T_bf
                    )

                cmd_msg = JointState()
                cmd_msg.name = JOINT_MOTOR_MAP.keys()
                cmd_msg.position = []
                for leg_name in ORDERED_LEG_NAME_ARRAY:
                    front = False
                    left = False
                    if leg_name[0] == "F":
                        front = True

                    if leg_name[1] == "L":
                        left = True

                    leg_ik_solution = self.ik_solver.solve_leg_ik(
                        gait_T_bf[leg_name.lower()][0, 3],
                        gait_T_bf[leg_name.lower()][1, 3],
                        gait_T_bf[leg_name.lower()][2, 3],
                        left,
                        front
                   )
                    cmd_msg.position += leg_ik_solution.tolist()
                    self.joint_control_pub.publish(cmd_msg)
                    
            rate.sleep()
