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
    states = ['stance','flight']
    transitions = [
        {'trigger':'touchdown', 'source': 'flight', 'dest': 'stance'},
        {'trigger':'liftoff', 'source': 'stance', 'dest': 'flight'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=Hopper.states, transitions=Hopper.transitions, initial='stance')
        self.motor_id = [1, 2,4]
        self.dt = .005
        self.contact_threshold = 10
        self.liftoff_threshold = 1
        self.last_error_for_derivative = [0,0]

        # model state
        self.q = [0, 0]
        self.orientation = [0,0,0]

        VI = VrepInterface(self.motor_id, self.dt, gyroscope=True, accelerometer=True, ft_sensor_names=['Force_sensor'])
        MotionManager.__init__(self, VI)

    def read_sensors(self, initialize=False):
        gyro_data = np.array(self.device.read_gyro(initialize))
        accel_data = np.array(self.device.read_accelerometer(initialize))
        force_data, torque_data = self.device.read_ft_sensor(0, initialize)
        return gyro_data, accel_data, np.array(force_data), np.array(torque_data)

    def update_state(self):
        gyro_data, accel_data, force_data, _  =self.read_sensors()

        self.total_foot_force = np.sum(force_data)
        self.orientation = [ori + 20*gyro*self.dt for ori, gyro in zip(self.orientation, gyro_data)]
        print "orientation: {},{},{}".format(*self.orientation)
        return gyro_data, accel_data, force_data, _




with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    h.read_sensors(initialize=True)
    h.advance_timestep()

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
        # gyro, acc, force, torque = h.read_sensors()
        gyro, acc, force, torque = h.update_state()


        # logging
        orientation_list.append((h.orientation))
        gyro_list.append(h.orientation)
        total_force = sum(force)

        # get positions
        h.q = h.get_all_current_position()
        actual_orientation_list.append(h.q)

        # command position
        h.set_all_command_position([h.orientation[1],0,np.pi/6*np.sin(10*t)], send=False)

    wait_for_input(1)

    ## plotting orientation_data
    t = np.arange(0,n_time_steps*h.dt,h.dt)
    x, y, z = zip(*orientation_list)
    gx,gy,gz = zip(*gyro_list)
    _, _, y_actual = zip(*actual_orientation_list)
    fig, ax = plt.subplots(nrows=2, ncols=1)

    ax[0].plot(t,x, label="x_orientation")
    ax[0].plot(t,y, label="y_orientation")
    ax[0].plot(t,z, label="z_orientation")
    ax[0].plot(t,y_actual, label="y actual orientation")
    ax[0].set_title('robot_orientation from gyro')

    ax[0].legend(loc=3)

    ax[1].plot(t,gx,label='gyro_x')
    ax[1].plot(t,gy,label='gyro_y')
    ax[1].plot(t,gz,label='gyro_z')
    ax[1].set_title('gyro_reading')
    ax[1].legend(loc=3)


    plt.show()
