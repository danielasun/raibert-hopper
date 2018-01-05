from rf.player.motion_manager import MotionManager
from rf.player.vrep.vrep_interface import VrepInterface
from transitions import Machine
from rf.util.misc import wait_for_input

class Hopper(MotionManager):
    states = ['stance','airborne']
    transitions = [
        {'trigger':'touchdown', 'source': 'airborne', 'dest': 'stance'},
        {'trigger':'liftoff', 'source': 'stance', 'dest': 'airborne'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=Hopper.states, transitions=Hopper.transitions, initial='airborne')

        self.motor_id = [1, 2, 3]
        self.dt = .005
        self.contact_threshold = 20
        self.liftoff_threshold = 10
        VI = VrepInterface(self.motor_id, self.dt, gyroscope=True, accelerometer=True, ft_sensor_names=['Force_sensor'])
        MotionManager.__init__(self, VI)


with Hopper() as h: # still works because Hopper inherits from MotionManager

    h.initialize()
    print h.state

    for i in xrange(500):
        h.advance_timestep()
        force = sum(h.device.read_ft_sensor(0))
        print force, h.state

        if h.state == 'airborne':
            h.set_command_position([0, 0, 0])
            if force > h.contact_threshold:
                h.trigger('touchdown')
                continue

        if h.state == 'stance':
            h.set_command_position([0,0,.1])
            if force < h.liftoff_threshold:
                h.trigger('liftoff')
                continue






    wait_for_input(3)



#
#
# motor_id = [1,2,3]
# dt = .01
# jump_timesteps = 60
# stance_timesteps = 20
# VI = VrepInterface(motor_id, dt, gyroscope=True, accelerometer=True)
# with MotionManager(VI) as MM:
#     MM.initialize()
#
#     for i in range(10):
#
#         for i in range(jump_timesteps):
#             # print "timestep"
#             # print MM.device.read_gyro()
#             # print MM.device.read_accelerometer()
#             MM.set_command_position([0,0,-0])
#             # MM.advance_timestep()
#         for i in range(stance_timesteps):
#             # print "timestep"
#             # print MM.device.read_gyro()
#             # print MM.device.read_accelerometer()
#             MM.set_command_position([0,0,-.1])
#             # MM.advance_timestep()
