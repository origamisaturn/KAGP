import numpy as np

class radialControl(om.ExplicitComponent):
    """
    Component containing radial rate control block.
    """

    def setup(self):
        """
        inputs:
            position
            velocity
            T_go
            thrust i guess
            a bunch of params
        outputs:
            angle
        """
        ...
    def setup_partials(self):
        self.declare_partials('*', '*', method='fd')
    def compute(self, inputs, outputs):
        r = np.array(self.inputs['r'])
        v = np.array(self.inputs['v'])
        T_go = self.inputs['T_go']
        ...
