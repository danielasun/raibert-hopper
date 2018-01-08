from transitions import Machine
from rf.player.motion_manager import MotionManager

class Hopper(MotionManager):
    states = ['flight', 'landing','compression','thrust','unloading']
    def __init__(self):
        self.machine = Machine(model=self, states=Hopper.states, initial='flight')

        # adding transitions
        self.machine.add_transition('touchdown', 'flight', 'landing')
        self.machine.add_transition('leg_shortens','landing','compression', )
        self.machine.add_transition('bottom','compression','thrust')
        self.machine.add_transition('leg_full_length','thrust','unloading')
        self.machine.add_transition('lift_off','unloading','flight')

    def landing(self):
        print "landing"

    def on_enter_landing(self):
        print "on_landing"


# test fsm
h = Hopper()
triggers = ['touchdown', 'leg_shortens','bottom','leg_full_length', 'lift_off']
for s in triggers:
    print h.state
    h.trigger(s)
print h.state

# test triggers
h=Hopper()
h.trigger('touchdown')
h.trigger('leg_shortens')
