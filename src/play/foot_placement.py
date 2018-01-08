from rf.player.motion_manager import MotionManager
from rf.player.vrep.vrep_interface import VrepInterface
from transitions import Machine
from rf.util.misc import wait_for_input
import rf.util.matrix as m
import numpy.matlib as np
import matplotlib.pyplot as plt

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
    states = ['flight', 'landing','compression','thrust','unloading']
    transitions = [
        ['touchdown','flight','landing'],
        ['leg_shortens','landing','compression'],
        ['bottom','compression','thrust'],
        ['leg_full_length','thrust','unloading'],
        ['lift_off','unloading','flight']
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=Hopper.states, transitions=Hopper.transitions, initial='flight')
        self.motor_id = [1, 2]
        self.dt = .002

        # parameter
        self.contact_threshold = 10
        self.liftoff_threshold = 1
        self.spring_compressed = -.024


        # model state
        self.q = [0, 0]
        self.body_vel = [0,0,0]
        self.body_rot_vel = [0, 0, 0]
        self.orientation = [0,0,0]
        self.foot_sensor_reading = 0

        VI = VrepInterface(self.motor_id, self.dt, gyroscope=True, accelerometer=True, ft_sensor_names=['Force_sensor'])
        MotionManager.__init__(self, VI)

    def read_sensors(self, initialize=False):
        gyro_data = np.array(self.device.read_gyro(initialize))
        accel_data = np.array(self.device.read_accelerometer(initialize))
        force_data, torque_data = self.device.read_ft_sensor(0, initialize)
        return gyro_data, accel_data, np.array(force_data), np.array(torque_data)

    def update_state(self):
        gyro_data, accel_data, force_data, _  = self.read_sensors()
        self.q = self.get_all_current_position()
        self.foot_sensor_reading = np.sum(force_data)
        self.orientation = [ori + 20*gyro*self.dt for ori, gyro in zip(self.orientation, gyro_data)]
        self.body_vel = [bd_vel + acc_i*self.dt**2 for bd_vel, acc_i in zip(self.body_vel, accel_data)]
        self.body_rot_vel = gyro_data
        print "orientation: {},{},{}".format(*self.orientation)
        return gyro_data, accel_data, force_data, _

with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    h.read_sensors(initialize=True)

    # set up for plotting
    orientation_list = []
    gyro_list = []
    actual_orientation_list = []
    n_time_steps = 500
    times = np.arange(0, n_time_steps * h.dt, h.dt)

    for t in times:
        h.advance_timestep()
        print h.state

        # read sensors
        gyro, acc, force, torque = h.read_sensors()
        h.update_state()
        orientation_list.append(h.orientation)

        total_force = sum(force)

        if h.state == 'flight':
            h.set_all_command_position([.1, 0], send=False)
            if total_force > h.contact_threshold:
                h.trigger('touchdown')
                continue

        if h.state == 'landing':
            # stop exhausting leg
            # zero hip torque

            # if leg shortens
            if h.
                h.trigger('compression')

        if h.state == 'thrust':
            # pressurize leg
            # servo body attitude with hip

        if h.state == 'unloading':
            # stop thrust
            # zero hip torque

        if h.state == 'flight':
            # exhaust leg to low pressure
            # position leg for landing





        if h.state == 'stance':
            h.set_all_command_position([h.orientation[1], .05,], send=False)
            if total_force < h.liftoff_threshold:
                h.trigger('liftoff')
                continue

    wait_for_input(1)