import rospy
from sensor_msgs.msg import Joy
from spotmicro_teleop.msg import SpotMicroTeleop

class Teleop(object):
    TRIANGLE = 2
    SQUARE = 3

    R_FB = 4
    R_LR = 3
    L_FB = 1
    L_LR = 0
    LB = 2
    RB = 5

    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_sub)
        self.teleop_pub = rospy.Publisher("spotmicro/teleop",
                                     SpotMicroTeleop,
                                     queue_size=1)

        self.r_fb = 0
        self.r_lr = 0
        self.l_fb = 0
        self.l_fr = 0

        self.estop = False
        self.in_place_deadman = False
        self.deadman = False

        self.deadman_thresh = 0.1
        
    def joy_sub(self, msg):
        if self.estop:
            self.estop = (msg.buttons[self.SQUARE] < 1)
        else:
            self.estop = (msg.buttons[self.TRIANGLE] > 0)

        self.in_place_deadman = msg.axes[self.LB] < self.deadman_thresh
        self.deadman = msg.axes[self.RB] < self.deadman_thresh

        self.r_fb = msg.axes[self.R_FB]
        self.r_lr = msg.axes[self.R_LR]
        self.l_fb = msg.axes[self.L_FB]
        self.l_lr = msg.axes[self.L_LR]

    def run(self):
        rate = rospy.Rate(50)
        teleop_msg = SpotMicroTeleop()
        while not rospy.is_shutdown():
            teleop_msg.in_place = self.in_place_deadman

            if self.in_place_deadman:
                teleop_msg.twist.angular.x = self.r_lr
                teleop_msg.twist.angular.y = self.r_fb
                teleop_msg.twist.angular.z = self.l_lr
                teleop_msg.twist.linear.z = self.l_fb
            elif self.deadman:
                teleop_msg.twist.linear.x = self.r_fb
                teleop_msg.twist.linear.y = self.r_lr
                teleop_msg.twist.angular.z = self.l_lr

            if self.deadman or self.in_place_deadman:
                self.teleop_pub.publish(teleop_msg)
            rate.sleep()
