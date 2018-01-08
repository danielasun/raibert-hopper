from transitions import Machine
from rf.player.motion_manager import MotionManager

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


# test fsm
h = Hopper()
triggers = ['touchdown', 'leg_shortens','bottom','leg_full_length', 'lift_off']
for s in triggers:
    print h.state
    h.trigger(s)
print h.state

# test triggers
h=Hopper()

