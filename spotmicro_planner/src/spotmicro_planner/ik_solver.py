import numpy as np


class SpotMicroIkSolver(object):
    def __init__(self,
                 l_shoulder,
                 l_upper_arm,
                 l_forearm,
                 l_x,
                 l_y,
                 f_x,
                 f_y,
                 f_z,
                 shoulder_lims,
                 knee_lims,
                 wrist_lims):
        self.l_shoulder = l_shoulder
        self.l_upper_arm = l_upper_arm
        self.l_forearm = l_forearm
        self.l_x = l_x
        self.l_y = l_y
        self.f_x = f_x
        self.f_y = f_y
        self.f_z = f_z

        self.T_fl = np.eye(4)
        self.T_flt = np.eye(4)
        self.T_fl[0][3] = -self.l_x / 2.0
        self.T_fl[1][3] = -self.l_y / 2.0
        self.T_flt[0][3] = -self.f_x / 2.0
        self.T_flt[1][3] = -self.f_y / 2.0
        self.T_flt[2][3] = self.f_z

        self.T_fr = np.eye(4)
        self.T_frt = np.eye(4)
        self.T_fr[0][0] = -1
        self.T_fr[1][1] = -1
        self.T_fr[0][3] = -self.l_x / 2.0
        self.T_fr[1][3] = self.l_y / 2.0
        self.T_frt[0][3] = -self.f_x / 2.0
        self.T_frt[1][3] = self.f_y / 2.0
        self.T_frt[2][3] = self.f_z

        self.T_rl = np.eye(4)
        self.T_rlt = np.eye(4)
        self.T_rl[0][3] = self.l_x / 2.0
        self.T_rl[1][3] = -self.l_y / 2.0
        self.T_rlt[0][3] = self.f_x / 2.0 + 0.03
        self.T_rlt[1][3] = -self.f_y / 2.0
        self.T_rlt[2][3] = self.f_z

        self.T_rr = np.eye(4)
        self.T_rrt = np.eye(4)
        self.T_rr[0][0] = -1
        self.T_rr[1][1] = -1
        self.T_rr[0][3] = self.l_x / 2.0
        self.T_rr[1][3] = self.l_y / 2.0
        self.T_rrt[0][3] = self.f_x / 2.0 + 0.03
        self.T_rrt[1][3] = self.f_y / 2.0
        self.T_rrt[2][3] = self.f_z

        self.shoulder_lims = shoulder_lims
        self.knee_lims = knee_lims
        self.wrist_lims = wrist_lims

    def validate_joint_limits(self, joint_angles, left_side, front):
        lf = "R"
        fr = "R"
        if left_side:
            lf = "L"

        if front:
            fr = "F"

        leg_key = fr + lf
        within_wrist_lims = (joint_angles[2] < self.wrist_lims[leg_key][1]) & \
                            (joint_angles[2] > self.wrist_lims[leg_key][0])

        within_knee_lims = (joint_angles[1] < self.knee_lims[leg_key][1]) & \
                            (joint_angles[1] > self.knee_lims[leg_key][0])

        within_shoulder_lims = (joint_angles[0] < self.shoulder_lims[leg_key][1]) & \
                               (joint_angles[0] > self.shoulder_lims[leg_key][0])

        return within_wrist_lims & within_knee_lims & within_shoulder_lims
        
    def solve_leg_ik(self, f_x, f_y, f_z, left_side, front):
        D = (f_x**2 + f_y**2 + f_z**2
             - self.l_shoulder**2 - self.l_upper_arm**2 - self.l_forearm**2) \
             / (2 * self.l_upper_arm * self.l_forearm)

        if np.abs(D) > 1:
            print("Leg domain is invalid %d" % D)
            D = np.clip(D, -1, 1)

        ik_solution = np.zeros(3)

        sq_comp = np.sqrt(f_y**2 + f_z**2 - self.l_shoulder**2)
            
        if left_side:
            ik_solution[2] = np.math.atan2(-np.sqrt(1 - D**2), D)
        else:
            ik_solution[2] = np.math.atan2(np.sqrt(1 - D**2), D)

        ik_solution[0] = np.math.atan2(f_z, f_y) + np.math.atan2(sq_comp, -self.l_shoulder)

        ik_solution[1] = np.math.atan2(f_x, sq_comp) \
                         - np.math.atan2(self.l_forearm * np.math.sin(ik_solution[2]), self.l_upper_arm
                                         + self.l_forearm * np.math.cos(ik_solution[2]))
        if not front:
            ik_solution[0] *= -1

        if not left_side:
            ik_solution[0] *= -1

        if not self.validate_joint_limits(ik_solution, left_side, front):
            print("IK solution out of the joint limits!")
            print(ik_solution)
            return None
        
        return ik_solution

    def solve_body_ik(self, b_x, b_y, b_z, b_rr, b_rp, b_ry):
        R_x = np.eye(4)
        R_x[1][1] = np.math.cos(b_rr)
        R_x[1][2] = np.math.sin(b_rr)
        R_x[2][1] = -np.math.sin(b_rr)
        R_x[2][2] = np.math.cos(b_rr)

        R_y = np.eye(4)
        R_y[0][0] = np.math.cos(b_rp)
        R_y[0][2] = np.math.sin(b_rp)
        R_y[2][0] = -np.math.sin(b_rp)
        R_y[2][2] = np.math.cos(b_rp)

        R_z = np.eye(4)
        R_z[0][0] = np.math.cos(b_ry)
        R_z[0][1] = -np.math.sin(b_ry)
        R_z[1][0] = np.math.sin(b_ry)
        R_z[1][1] = np.math.cos(b_ry)

        T_rot = np.dot(np.dot(R_z, R_y), R_x)
        T_tran = np.eye(4)
        T_tran[0][3] = b_x
        T_tran[1][3] = b_y
        T_tran[2][3] = b_z

        T_tot = np.dot(T_rot, T_tran)

        T_new_fl = T_tot.dot(self.T_fl)
        T_flt_rel = np.dot(np.linalg.inv(T_new_fl), self.T_flt)

        T_new_fr = T_tot.dot(self.T_fr)
        T_frt_rel = np.dot(np.linalg.inv(T_new_fr), self.T_frt)

        T_new_rl = T_tot.dot(self.T_rl)
        T_rlt_rel = np.dot(np.linalg.inv(T_new_rl), self.T_rlt)

        T_new_rr = T_tot.dot(self.T_rr)
        T_rrt_rel = np.dot(np.linalg.inv(T_new_rr), self.T_rrt)

        ik_solution = dict()
        ik_solution["fl"] = self.solve_leg_ik(T_flt_rel[0][3],
                                              T_flt_rel[1][3],
                                              T_flt_rel[2][3],
                                              True,
                                              True)

        ik_solution["fr"] = self.solve_leg_ik(T_frt_rel[0][3],
                                              T_frt_rel[1][3],
                                              T_frt_rel[2][3],
                                              False,
                                              True)

        ik_solution["rl"] = self.solve_leg_ik(T_rlt_rel[0][3],
                                              T_rlt_rel[1][3],
                                              T_rlt_rel[2][3],
                                              True,
                                              False)

        ik_solution["rr"] = self.solve_leg_ik(T_rrt_rel[0][3],
                                              T_rrt_rel[1][3],
                                              T_rrt_rel[2][3],
                                              False,
                                              False)

        # Current foot transform relative to the body
        T_bf = dict()
        T_bf["fl"] = T_flt_rel
        T_bf["fr"] = T_frt_rel
        T_bf["rl"] = T_rlt_rel
        T_bf["rr"] = T_rrt_rel

        return ik_solution, T_bf
