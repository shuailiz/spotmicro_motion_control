import numpy as np
import time
import copy
from spotmicro_motor_control.constants import ORDERED_LEG_NAME_ARRAY


def get_leg_id_from_name(leg_name):
    return ORDERED_LEG_NAME_ARRAY.index(leg_name.upper())

class BezierGenerator(object):
    def __init__(self, num_bezier_points):
        self.NumBezierPoints = num_bezier_points

    def BezierPoint(self, t, k, point):
        """Calculate the point on the bezier curve
           based on phase (0->1), point number (0-11),
           and the value of the point itself

           :param t: phase
           :param k: point number
           :param point: point value
           :return: Value through Bezier Curve
        """
        return point * self.Binomial(k) * np.power(t, k) * np.power(
            1 - t, self.NumBezierPoints - k)

    def Binomial(self, k):
        """Solves the binomial theorem given a Bezier point number
           relative to the total number of Bezier points.

           :param k: Bezier point number
           :returns: Binomial solution
        """
        return np.math.factorial(self.NumBezierPoints) / (
            np.math.factorial(k) * np.math.factorial(self.NumBezierPoints - k))


class GaitGenerator(object):
    STANCE = 0
    SWING = 1
    def __init__(self,
                 L,
                 clearance_height,
                 num_bezier_points,
                 lateral_fraction,
                 stance_delta=0.005,
                 Sref=[0, 0.2, 0.45, 0.7],
                 v_d=0.1,
                 Tswing=0.2):
        self.b_generator = BezierGenerator(num_bezier_points)
        self.L = L
        self.clearance_height = clearance_height
        self.lateral_fraction = lateral_fraction
        self.stance_delta = stance_delta
        
        self.Sref = Sref
        self.ref_leg_ind = 0
        self.modulated_rotation = [0, 0, 0, 0]

        self.Tswing = Tswing
        self.Tstance = 2 * L / v_d

        self.time_since_last_TD = 0
        self.time_ref_TD = time.time()
        
        self.TD = False
        self.SwRef = 0

        # Swing Bezier curve points
        self.swing_step_control = np.array([
            -1,  # Ctrl Point 0, half of stride len
            -1.4,  # Ctrl Point 1 diff btwn 1 and 0 = x Lift vel
            -1.5,  # Ctrl Pts 2, 3, 4 are overlapped for
            -1.5,  # Direction change after
            -1.5,  # Follow Through
            0.0,  # Change acceleration during Protraction
            0.0,  # So we include three
            0.0,  # Overlapped Ctrl Pts: 5, 6, 7
            1.5,  # Changing direction for swing-leg retraction
            1.5,  # requires double overlapped Ctrl Pts: 8, 9
            1.4,  # Swing Leg Retraction Velocity = Ctrl 11 - 10
            1
        ])

        self.swing_height_control = np.array([
            0.0,  # Double Overlapped Ctrl Pts for zero Lift
            0.0,  # Veloicty wrt hip (Pts 0 and 1)
            0.9,  # Triple overlapped control for change in
            0.9,  # Force direction during transition from
            0.9,  # follow-through to protraction (2, 3, 4)
            0.9,  # Double Overlapped Ctrl Pts for Traj
            0.9,  # Dirctn Change during Protraction (5, 6)
            1.1,  # Maximum Clearance at mid Traj, Pt 7
            1.1,  # Smooth Transition from Protraction
            1.1,  # To Retraction, Two Ctrl Pts (8, 9)
            0.0,  # Double Overlap Ctrl Pts for 0 Touchdown
            0.0,  # Veloicty wrt hip (Pts 10 and 11)
        ])

    def compute_swing_xyz_bpoints(self,
                                  L,
                                  lateral_fraction):
        x_polar = np.cos(lateral_fraction)
        y_polar = np.sin(lateral_fraction)
        step_bpoints = self.swing_step_control * L
        height_bpoints = self.swing_height_control * self.clearance_height
        x_bpoints = step_bpoints * x_polar
        y_bpoints = step_bpoints * y_polar
        z_bpoints = height_bpoints
        return x_bpoints, y_bpoints, z_bpoints
        
    def get_ti(self, leg_ind):
        phase_lag = 0
        phase_lag = self.Sref[leg_ind]
        return self.time_since_last_TD - phase_lag * (self.Tswing + self.Tstance)


    def get_phase(self, leg_ind):
        Tstride = self.Tswing + self.Tstance
        ti = self.get_ti(leg_ind)

        cur_phase = 0
        phase_type = self.STANCE
        
        if ti < -self.Tswing:
            ti += Tstride

        if ti >= 0 and ti <= self.Tstance:
            phase_type = self.STANCE
            cur_phase = ti / self.Tstance
        elif ti >= -self.Tswing and ti < 0:
            phase_type = self.SWING
            cur_phase = (ti + self.Tswing) / self.Tswing
        elif ti > self.Tstance and ti <= Tstride:
            phase_type = self.SWING
            cur_phase = (ti - self.Tstance) / self.Tswing
        else:
            print("Invalid ti %f found for leg %d" % (ti, leg_ind))
            
        if cur_phase > 1:
            cur_phase = 1

        if leg_ind == self.ref_leg_ind:
            if phase_type == self.SWING:
                self.SwRef = cur_phase
                if self.SwRef >= 0.99:
                    self.set_touchdown()
            else:
                self.SwRef = 0
            
        return phase_type, cur_phase

    def set_touchdown(self):
        self.time_ref_TD = time.time()
        self.TD = True
    
    def update(self):
        if self.SwRef >= 0.95 and self.TD:
            self.time_ref_TD = time.time()
            self.TD = False
            self.SwRef = 0

        Tstride = self.Tswing + self.Tstance
        self.time_since_last_TD = time.time() - self.time_ref_TD
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0:
            self.time_since_last_TD = 0

    def generate_swing_coord(self,
                             phase,
                             L,
                             lateral_fraction):
        step_x = 0
        step_y = 0
        step_z = 0

        x_bpoints, y_bpoints, z_bpoints = self.compute_swing_xyz_bpoints(
            L,
            lateral_fraction
        )

        for i in range(len(x_bpoints)):
            step_x += self.b_generator.BezierPoint(phase, i, x_bpoints[i])
            step_y += self.b_generator.BezierPoint(phase, i, y_bpoints[i])
            step_z += self.b_generator.BezierPoint(phase, i, z_bpoints[i])
        return step_x, step_y, step_z

    def generate_stance_coord(self,
                              phase,
                              L,
                              lateral_fraction):
        x_polar = np.cos(lateral_fraction)
        y_polar = np.sin(lateral_fraction)

        step = L * (1 - 2.0 * phase)
        step_x = step * x_polar
        step_y = step * y_polar
        if L != 0:
            step_z = -self.stance_delta * np.cos(
                np.pi * step / 2 / L
            )
        else:
            step_z = 0

        return step_x, step_y, step_z

    def generate_step(self,
                      phase,
                      lateral_fraction,
                      yaw_rate,
                      T_bf,
                      leg_ind,
                      step_type):
        nominal_bodytofoot_mag = np.sqrt(T_bf[0, 3]**2 +\
                                         T_bf[1, 3]**2)
        nominal_bodytofoot_direction = np.arctan2(T_bf[1, 3],
                                                  T_bf[0, 3])

        foot_arc_angle = np.pi / 2.0 +\
                         nominal_bodytofoot_direction + \
                         self.modulated_rotation[leg_ind]

        if step_type == self.SWING:
            x_delta_lin,\
            y_delta_lin,\
            z_delta_lin = self.generate_swing_coord(phase,
                                                    self.L,
                                                    lateral_fraction)

            x_delta_rot,\
            y_delta_rot,\
            z_delta_rot = self.generate_swing_coord(phase,
                                                    yaw_rate,
                                                    foot_arc_angle)
        elif step_type == self.STANCE:
            x_delta_lin,\
            y_delta_lin,\
            z_delta_lin = self.generate_stance_coord(phase,
                                                     self.L,
                                                     lateral_fraction)

            x_delta_rot,\
            y_delta_rot,\
            z_delta_rot = self.generate_stance_coord(phase,
                                                     yaw_rate,
                                                     foot_arc_angle)
        else:
            print("Invalid step type!")
    

        modulate_bodytofoot_mag = np.sqrt((x_delta_lin + x_delta_rot) ** 2 +\
                                          (y_delta_lin + y_delta_rot) ** 2)

        mod = np.arctan2(modulate_bodytofoot_mag,
                         nominal_bodytofoot_mag)
        self.modulated_rotation[leg_ind] = mod

        #print("%s -> x_step: %f and %s phase is %f"%(ORDERED_LEG_NAME_ARRAY[leg_ind], x_delta_lin, step_type, phase))
        step_coord = np.array([
            x_delta_lin + x_delta_rot,
            y_delta_lin + y_delta_rot,
            z_delta_lin + z_delta_rot
        ])
        return step_coord

    def update_speed(self, speed):
        if speed != 0:
            self.Tstance = 2.0 * abs(self.L) / abs(speed)
            self.L = abs(self.L) * abs(speed) / speed
        else:
            self.Tstance = 0
            self.TD = False
            self.time_since_last_TD = 0

        # According to the spotmicro repo, this makes the gait more stable
        if self.Tstance > 4 * self.Tswing:
            self.Tstance = 8 * self.Tswing
    
    def generate_step_traj(self,
                           speed,
                           yaw_rate,
                           lateral_fraction,
                           T_bf,
                           contacts=[0, 0, 0, 0],
                           L=None,
                           clearance_height=None
    ):
        if L is not None:
            self.L = L

        if clearance_height is not None:
            self.clearance_height = clearance_height

        if abs(lateral_fraction) > 1:
            print("Lateral fraction has to be between -1 and 1!")
            lateral_fraction = 1 * abs(lateral_fraction) / lateral_fraction

        self.update_speed(speed)

        if contacts[0] == 1:
            self.set_touchdown()

        self.update()

        T_bf_generated = copy.deepcopy(T_bf)

        axis_sign = [1, 1, 1]
        for (leg_name, Tbf) in T_bf.items():
            leg_id = get_leg_id_from_name(leg_name)
            if self.Tstance > 0:
                phase_type, cur_phase = self.get_phase(leg_id)
                step_coord = self.generate_step(
                    cur_phase,
                    lateral_fraction,
                    yaw_rate,
                    Tbf,
                    leg_id,
                    phase_type
                )
            else:
                step_coord = np.zeros(3)

            if leg_name[1] != 'l':
                axis_sign[0] = -1
            else:
                axis_sign[0] = 1
            for i in range(3):
                T_bf_generated[leg_name][i, 3] = Tbf[i, 3] + step_coord[i] * axis_sign[i]

        return T_bf_generated
            
        
        
