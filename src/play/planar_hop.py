from rf.player.motion_manager import MotionManager
from rf.player.vrep.vrep_interface import VrepInterface
from transitions import Machine
from rf.util.misc import wait_for_input
import rf.util.matrix as m
import numpy.matlib as np


"""
   Hopping height control

   Each fixed amount of thrust results in a certain hopping height for the robot

   e_prev = e
   e = desired_height - actual_height
   de = (e - e_prev)/dt
   Leg Thrust = kp_thrust*e + kd_thrust*de 

   """
"""

Velocity control

Each foot placement results in a certain forward speed for the robot

e_prev = e
e = desired_speed - actual_speed
de = (e - e_prev)/dt
Foot distance = kp_vel*e_vel + kd_vel*de + xd*Ts/(2)
desired leg angle = body_attitude - asin(foot_distance/r)

"""
"""

Control Body Attitude

body torque = -kp(attitude - desired attitude) - kv(d attitude)

"""
"""
Parameters
stance time, Ts
desired speed
desired jumping height

Data required:
body attitude
linear velocity
leg length
"""

class Hopper(MotionManager):
    states = ['stance','flight']
    transitions = [
        {'trigger':'touchdown', 'source': 'flight', 'dest': 'stance'},
        {'trigger':'liftoff', 'source': 'stance', 'dest': 'flight'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=Hopper.states, transitions=Hopper.transitions, initial='stance')
        self.motor_id = [1, 2]
        self.dt = .005
        self.contact_threshold = 10
        self.liftoff_threshold = 1
        self.q = [0,0]
        self.last_error_for_derivative = [0,0]

        # self.acc = np.array([0,0,0])
        # self.ang_acc = np.array([0,0,0])
        # self.vel = np.array([0,0,0])
        # self.w = np.array([0,0,0]).transpose()

        VI = VrepInterface(self.motor_id, self.dt, gyroscope=True, accelerometer=True, ft_sensor_names=['Force_sensor'])
        MotionManager.__init__(self, VI)

    def calc_servo_command(self, des):
        q =  self.get_current_position()
        last_q = self.q
        self.q = q
        error = [des_i-q_i for des_i, q_i in zip(des, q) ]
        d_error = [error_i - last_error_i for error_i, last_error_i in zip(error, self.last_error_for_derivative)]
        self.last_error_for_derivative = error

        hip_torque = .0003*(error[0])/self.dt + 0*d_error[0]/self.dt
        leg_torque = 0*(error[1])/self.dt + 0*d_error[1]/self.dt
        print "hip_torque: {} leg_torque: {} ".format(hip_torque, leg_torque)
        return hip_torque, leg_torque

    def actuate(self, desired_angles, send=False):
        hip_torque, leg_torque = self.calc_servo_command(desired_angles)
        command = (hip_torque, leg_torque)
        self.set_joint_effort(command, send=False)

    def read_sensors(self, initialize=False):
        gyro_data = np.array(self.device.read_gyro(initialize))
        accel_data = np.array(self.device.read_accelerometer(initialize))
        force_data, torque_data = self.device.read_ft_sensor(0, initialize)
        return gyro_data, accel_data, np.array(force_data), np.array(torque_data)


with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    h.read_sensors(initialize=True)
    h.advance_timestep()

    for mm in xrange(1000):
        h.advance_timestep()
        print h.state
        gyro, acc, force, torque = h.read_sensors()
        total_force = sum(force)
        print force

        if h.state == 'flight':
            h.set_command_position([0, 0], send=False)
            h.advance_timestep()
            if total_force > h.contact_threshold:
                h.trigger('touchdown')
                continue

        if h.state == 'stance':
            h.set_command_position([0,.05], send=False)
            h.advance_timestep()
            if total_force < h.liftoff_threshold:
                h.trigger('liftoff')
                continue






    wait_for_input(1)