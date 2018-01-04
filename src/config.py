from rf.player.dynamixel_sdk.dxl_interface import DxlPort, DxlInterface
from rf.player.vrep.vrep_interface import VrepInterface

def player_initializer(args):
    # pass in arguments from util.misc.player_arg_parser

    motor_id = [1,2,3]
    Devices = []

    if args.simulation:
        print "simulating using VREP"
        VI = VrepInterface(motor_id, dt)
        Devices.append(VI)
        player = 'vrep'
        player_offset = VrepAngleOffset
    return Devices, player, player_offset
