from rf.player.motion_manager import MotionManager
from rf.player.vrep.vrep_interface import VrepInterface
from transitions import Machine
from rf.util.misc import wait_for_input
import rf.util.matrix as m
import numpy.matlib as np

class Hopper(MotionManager):
    states = ['stance','airborne']
    transitions = [
        {'trigger':'touchdown', 'source': 'airborne', 'dest': 'stance'},
        {'trigger':'liftoff', 'source': 'stance', 'dest': 'airborne'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=Hopper.states, transitions=Hopper.transitions, initial='stance')
        self.motor_id = [1, 2, 3]
        self.dt = .005
        self.contact_threshold = 10
        self.liftoff_threshold = 1
        self.acc = np.array([0,0,0])
        self.ang_acc = np.array([0,0,0])
        self.vel = np.array([0,0,0])
        self.w = np.array([0,0,0]).transpose()

        VI = VrepInterface(self.motor_id, self.dt, gyroscope=True, accelerometer=True, ft_sensor_names=['Force_sensor'])
        MotionManager.__init__(self, VI)

    def actuate(self, command, send=False):

        self.device.set_command_position((self.device.joint[0], self.device.joint[1]), (command[0], command[1]), send=False)
        print command[-1]
        self.device.set_joint_effort([2], [command[-1]], send=False)

    def read_sensors(self, initialize=False):
        gyro_data = np.array(self.device.read_gyro(initialize))
        accel_data = np.array(self.device.read_accelerometer(initialize))
        force_data, torque_data = self.device.read_ft_sensor(0, initialize)
        return gyro_data, accel_data, np.array(force_data), np.array(torque_data)

    # def update(self):
    #     print "update"
    #     gyr, acc, force = self.read_sensors()
    #     self.acc = acc
    #     # self.ang_acc = (self.w - gyr)*(1/self.dt)

        # print "accel: {}{}{}".format(*self.acc) + "angular_accel: {}{}{}".format(*self.ang_acc)

    # def update_orientation(self):
    #     gyro, acc, force = self.read_sensors()
    #     self.vel = self.vel + acc*self.dt


with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    h.read_sensors(initialize=True)
    # h.advance_timestep()
    # h.device.
    print h.state

    for i in xrange(500):

        gyro, acc, force, torque = h.read_sensors()
        total_force = sum(force)
        # print "gyro: {: 5.3f},{: 5.3f},{: 5.3f} ".format(*gyro) + "acc: {: 5.3f},{: 5.3f},{: 5.3f} ".format(*acc) + "total_force: {: 5.3f}".format(total_force)
        # h.update()
        print h.state

        if h.state == 'airborne':
            h.actuate([0, 0, -1], send=False)
            h.advance_timestep()
            if total_force > h.contact_threshold:
                h.trigger('touchdown')
                continue

        if h.state == 'stance':
            h.actuate([0,0,210], send=False)
            h.advance_timestep()
            if total_force < h.liftoff_threshold:
                h.trigger('liftoff')
                continue






    wait_for_input(1)