from rf.player.motion_manager import MotionManager
from rf.player.vrep.vrep_interface import VrepInterface

motor_id = [1,2,3]
dt = .01
jump_timesteps = 60
stance_timesteps = 20
VI = VrepInterface(motor_id, dt, gyroscope=True, accelerometer=True)
with MotionManager(VI) as MM:
    MM.initialize()

    for i in range(10):

        for i in range(jump_timesteps):
            # print "timestep"
            # print MM.device.read_gyro()
            # print MM.device.read_accelerometer()
            MM.set_command_position([0,0,-0])
            # MM.advance_timestep()
        for i in range(stance_timesteps):
            # print "timestep"
            # print MM.device.read_gyro()
            # print MM.device.read_accelerometer()
            MM.set_command_position([0,0,-.1])
            # MM.advance_timestep()
