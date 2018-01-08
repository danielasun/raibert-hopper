from rf.player.motion_manager import MotionManager
from rf.player.vrep.vrep_interface import VrepInterface
from transitions import Machine
from rf.util.misc import wait_for_input
import rf.util.matrix as m
import numpy.matlib as np
import matplotlib.pyplot as plt
import rf.player.vrep.vrep as vrep
from math import asin
from rf.util.mathfcn import wrap_between_pi_and_neg_pi
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

# TODO: the body stabilizer isn't working

class Hopper(MotionManager):
    states = ['flight', 'landing','compression','thrust','unloading']
    def __init__(self):
        self.machine = Machine(model=self, states=Hopper.states, initial='flight')

        # adding transitions
        self.machine.add_transition('touchdown', 'flight', 'landing')
        self.machine.add_transition('leg_shortens','landing','compression' )
        self.machine.add_transition('bottom','compression','thrust')
        self.machine.add_transition('stance_time_exceeded','thrust','unloading')
        self.machine.add_transition('liftoff',['landing','compression','thrust'],'unloading')
        self.machine.add_transition('unloading_time_exceeded', 'unloading', 'flight')

        # player interface
        self.motor_id = [1, 2]
        self.dt = .002
        VI = VrepInterface(self.motor_id, self.dt, gyroscope=True, accelerometer=True, ft_sensor_names=['Force_sensor'])
        MotionManager.__init__(self, VI)
        self.body_handle = VI.get_object_handles('1D_Hopper')

        # parameter
        self.contact_threshold = 10
        self.liftoff_threshold = 1
        self.spring_compressed = -.024
        self.unloading_time = .01 # time to wait before entering flight phase
        self.Ts = .05 # duration of stance phase = compression + thrust
        self.timer = 0
        self.desired_x_vel = .1 #m/s

        self.k_xdot = .01
        self.r = .2872
        self.thrust_length = .02

        # model state
        self.q = [0, 0]
        self.body_vel = [0,0,0]
        self.body_rot_vel = [0, 0, 0]
        self.orientation = [0,0,0]
        self.body_accel = [0,0,0]
        self.foot_total_force = 0



    def on_enter_compression(self):
        # zero thrust timer
        print __name__
        self.timer = 0

    def on_enter_unloading(self):
        self.timer = 0


    def read_sensors(self, initialize=False):
        gyro_data = np.array(self.device.read_gyro(initialize))
        accel_data = np.array(self.device.read_accelerometer(initialize))
        force_data, torque_data = self.device.read_ft_sensor(0, initialize)

        # cheating for now with the linear and angular velocity
        if initialize:
            opmode = vrep.simx_opmode_streaming
        else:
            opmode = vrep.simx_opmode_buffer
        _, body_lin_vel, body_ang_vel = vrep.simxGetObjectVelocity(self.device._sim_Client_ID, self.body_handle, opmode) # this needs to be relative to the boom attachment.

        return gyro_data, accel_data, np.array(force_data), np.array(torque_data), body_lin_vel, body_ang_vel

    def update_state(self):
        gyro_data, accel_data, force_data, _, body_lin_vel, body_ang_vel = self.read_sensors()
        self.q = self.get_all_current_position()
        self.foot_total_force = np.sum(force_data)
        self.orientation = [wrap_between_pi_and_neg_pi(ori + 20*gyro*self.dt) for ori, gyro in zip(self.orientation, gyro_data)] # todo: this line is suspect
        self.body_vel = body_lin_vel#[bd_vel + acc_i*self.dt for bd_vel, acc_i in zip(self.body_vel, accel_data)]
        self.body_rot_vel = gyro_data
        # print "orientation: {},{},{}".format(*self.orientation)
        return gyro_data, accel_data, force_data, _

    def calc_leg_landing_angle(self):
        xvel = self.body_vel[0]
        xf = xvel*self.Ts/2.0 + self.k_xdot*(xvel - self.desired_x_vel)
        hip_angle = self.orientation[1] - asin(xf/self.r)
        return hip_angle

    def get_body_velocity(self, initialize=False):
        # cheating for now
        if initialize:
            opmode = vrep.simx_opmode_streaming
        else:
            opmode = vrep.simx_opmode_buffer
        _, lin_vel, ang_vel = vrep.simxGetObjectVelocity(self.device._sim_Client_ID, self.body_handle, opmode)
        return lin_vel, ang_vel


with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    h.read_sensors(initialize=True)

    # set up for plotting
    orientation_list = []
    gyro_list = []
    actual_orientation_list = []
    body_vel_list = []
    n_time_steps = 1000
    times = np.arange(0, n_time_steps * h.dt, h.dt)

    for t in times:
        h.advance_timestep()
        print h.state

        # read sensors
        h.update_state()
        orientation_list.append(h.orientation)
        gyro_list.append(h.body_rot_vel)
        body_vel_list.append(h.body_vel)

        if h.foot_total_force < h.liftoff_threshold and h.state != 'flight' and h.state != 'unloading':
            h.trigger('liftoff')
            print "trigger: liftoff"


        if h.state == 'flight':
            # exhaust leg to low pressure
            # position leg for landing
            landing_angle = h.calc_leg_landing_angle()
            h.set_all_command_position([landing_angle, 0], send=False)
            if h.foot_total_force > h.contact_threshold:
                h.trigger('touchdown')
                print "trigger: touchdown"
                continue

        if h.state == 'landing':
            # stop exhausting leg
            # zero hip torque
            # if leg shortens
            h.trigger('leg_shortens')

        if h.state == 'compression':
            # upper leg chamber sealed
            # servo body attitude with hip
            h.set_all_command_position((h.orientation[1], 0), send=False)
            h.timer += h.dt

            if h.body_vel[2] > 0: # TODO: this isn't reading correctly, need to use something besides the IMU
                h.trigger('bottom')
                print "body_vel is positive"
                continue


        if h.state == 'thrust':
            # pressurize leg
            # servo body attitude with hip
            h.set_all_command_position((h.orientation[1], h.thrust_length), send=False)
            h.timer += h.dt
            if h.timer > h.Ts:
                h.trigger('stance_time_exceeded')
                print "trigger: stance time exceeded"
                continue


        if h.state == 'unloading':
            # stop thrust
            # zero hip torque
            h.set_all_command_position((h.orientation[1], 0), send=False)
            h.timer += h.dt
            if h.timer > h.Ts:
                h.trigger('unloading_time_exceeded')
                print "trigger: unloading time exceeded"
                continue






        #
        # if h.state == 'stance':
        #     h.set_all_command_position([h.orientation[1], .05,], send=False)


    wait_for_input(1)





    plt.show()

    ## plotting orientation_data
    # t = np.arange(0,n_time_steps*h.dt,h.dt)
    t = times
    x, y, z = zip(*orientation_list)
    gx,gy,gz = zip(*gyro_list)
    vx,vy,vz = zip(*body_vel_list)
    fig, ax = plt.subplots(nrows=3, ncols=1)

    ax[0].plot(t,x, label="x_orientation")
    ax[0].plot(t,y, label="y_orientation")
    ax[0].plot(t,z, label="z_orientation")
    ax[0].set_title('robot_orientation from gyro')

    ax[0].legend(loc=3)

    ax[1].plot(t,gx,label='gyro_x')
    ax[1].plot(t,gy,label='gyro_y')
    ax[1].plot(t,gz,label='gyro_z')
    ax[1].set_title('gyro_reading')
    ax[1].legend(loc=3)

    ax[2].plot(t, vx, label="v_velocity")


    plt.show()