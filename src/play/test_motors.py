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
        self.machine.add_transition('half_of_stance_time_exceeded', 'compression', 'thrust')
        self.machine.add_transition('stance_time_exceeded','thrust','unloading')
        self.machine.add_transition('liftoff',['landing','compression','thrust'],'unloading')
        self.machine.add_transition('unloading_time_exceeded', 'unloading', 'flight')

        # player interface
        self.motor_id = [1, 2]
        self.dt = .001
        VI = VrepInterface(self.motor_id, self.dt, gyroscope=True, accelerometer=True, ft_sensor_names=['Force_sensor'])
        MotionManager.__init__(self, VI)
        self.body_handle = VI.get_object_handles('1D_Hopper')
        self.body_ref_sphere_handle = VI.get_object_handles('Sphere')

        # vrep force control properties
        self.device.revolute_joint_torque_control_max_speed = 100 # rad/s
        self.device.prismatic_joint_torque_control_max_speed = 10 # m/s


        # parameter
        self.contact_threshold = 5
        self.liftoff_threshold = 1
        self.spring_compressed = -.024
        self.unloading_time = .02 # time to wait before entering flight phase
        self.Ts = .1 # duration of stance phase = compression + thrust
        self.timer = 0
        self.desired_x_vel = .1 # m/s
        self.r = .2872


        self.hopping_height_desired = .4

        self.thrust_length = .06
        self.leg_thrust = 1000 # [N]

        # gains
        self.k_xdot = .03
        self.kp_hopping_height = .4

        # model state
        self.q = [0, 0]
        self.body_lin_vel = [0, 0, 0]
        self.body_rot_vel = [0, 0, 0]
        self.body_euler_angles = [0, 0, 0]
        self.body_accel = [0,0,0]
        self.hopping_height = .4 # starting height [m]
        self.body_position = [0,.3,self.hopping_height]
        self.foot_total_force = 0
        self.going_up = False

        # errors


    def on_enter_compression(self):
        # zero thrust timer
        print __name__
        self.timer = 0

    def on_enter_unloading(self):
        self.timer = 0

    def on_enter_flight(self):
        self.going_up = True

    def read_sensors(self, initialize=False):
        gyro_data = np.array(self.device.read_gyro(initialize))
        accel_data = np.array(self.device.read_accelerometer(initialize))
        force_data, torque_data = self.device.read_ft_sensor(0, initialize)

        # cheating for now with values straight from VREP
        if initialize:
            opmode = vrep.simx_opmode_oneshot_wait
        else:
            opmode = vrep.simx_opmode_blocking

        _, body_lin_vel, body_ang_vel = vrep.simxGetObjectVelocity(self.device._sim_Client_ID, self.body_handle, opmode) # this needs to be relative to the boom attachment.
        _, euler_angles = vrep.simxGetObjectOrientation(self.device._sim_Client_ID, self.body_handle, self.body_ref_sphere_handle, opmode)
        _, body_position = vrep.simxGetObjectPosition(self.device._sim_Client_ID, self.body_handle, -1, opmode)

        return gyro_data, accel_data, np.array(force_data), np.array(torque_data), body_lin_vel, body_ang_vel, euler_angles, body_position

    def update_state(self):
        gyro_data, accel_data, force_data, _, body_lin_vel, body_ang_vel, body_euler_angles, body_position = self.read_sensors()
        self.q = self.get_all_current_position()
        self.foot_total_force = np.sum(force_data)
        self.body_euler_angles = body_euler_angles #[wrap_between_pi_and_neg_pi(ori + 10*gyro*self.dt) for ori, gyro in zip(self.body_euler_angles, gyro_data)]
        self.body_lin_vel = body_lin_vel#[bd_vel + acc_i*self.dt for bd_vel, acc_i in zip(self.body_vel, accel_data)]
        self.body_rot_vel = gyro_data
        self.body_position = body_position
        # print "body_euler_angles: {},{},{}".format(*self.body_euler_angles)
        return gyro_data, accel_data, force_data, _

    def calc_leg_landing_angle(self):
        xvel = self.body_lin_vel[0]
        self.xvel_err = xvel - self.desired_x_vel
        xf = xvel*self.Ts/2.0 + self.k_xdot*(self.xvel_err)
        hip_angle = -self.body_euler_angles[1] - asin(xf / self.r)
        return hip_angle

    def get_body_velocity(self, initialize=False):
        # cheating for now
        if initialize:
            opmode = vrep.simx_opmode_streaming
        else:
            opmode = vrep.simx_opmode_buffer
        _, lin_vel, ang_vel = vrep.simxGetObjectVelocity(self.device._sim_Client_ID, self.body_handle, opmode)
        return lin_vel, ang_vel

    def update_stance_time(self): # TODO: add kv error terms
        # control hopping height by decreasing support time TODO: this doesn't work, move piston to be force controlled
        self.Ts += self.kp_hopping_height*(self.hopping_height_desired - self.hopping_height)
        print "support time: {}".format(self.Ts)

    def actuate_joints(self, command, send=False):
        self.set_command_position([0], [command[0]], send)
        self.set_joint_effort([1], [command[1]], send)



with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    h.read_sensors(initialize=True)

    for i in xrange(500):
        h.actuate_joints([0,5])
        h.advance_timestep()
    for i in xrange(500):
        h.actuate_joints([0,-5])
        h.advance_timestep()
    for i in xrange(500):
        h.actuate_joints([0,10])
        h.advance_timestep()