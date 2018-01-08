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
        self.device.set_all_command_position(self.device.joint[:2], command[:2], send=False)
        self.device.set_joint_effort([2], [command[2]], send=False)

    def read_sensors(self, initialize=False):
        gyro_data = np.array(self.device.read_gyro(initialize))
        accel_data = np.array(self.device.read_accelerometer(initialize))
        force_data, torque_data = self.device.read_ft_sensor(0, initialize)
        return gyro_data, accel_data, np.array(force_data), np.array(torque_data)

with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    h.read_sensors(initialize=True)
    # h.advance_timestep()
    # h.device.
    print h.state
    for i in xrange(500):
        h.actuate([0,0,50*np.sin(2*i/100.*np.pi)+100], send=False)
        h.advance_timestep()



