from rf.player.motion_manager import MotionManager
from rf.player.vrep.vrep_interface import VrepInterface
from rf.util.misc import wait_for_input
import numpy as np

VI = VrepInterface([1],dt=.005)
with MotionManager(VI) as MM:
    MM.initialize()

    print MM.device.joint
    ntimes = 1000
    t = np.linspace(0,6*np.pi,ntimes)
    cmd_list = [9,10.5,-3,5,10]
    ind = 0
    for i in xrange(1,ntimes):

        if i%200 ==0:
            ind += 1
        cmd = cmd_list[ind]
        joint_effort = MM.get_joint_effort(ids=[0])


        MM.set_joint_effort([0],[cmd])
        print 'joint effort: {}, cmd: {}'.format(joint_effort,cmd)
        MM.advance_timestep()
    wait_for_input(3)