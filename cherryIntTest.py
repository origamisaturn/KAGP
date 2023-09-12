import unittest
from cherryInt import rocket_ode
import math

class TestRocketOde(unittest.TestCase):
    def setUp(self):
        Isp = 300
        F_thrust_max = 100 #
        self.lunar_radius = 1740e3   # m
        self.lunar_mu = 4.9048695e12 # m^3/s^2
        self.mu_earth = 3.986004418e14
        self.earth_radius = 6378.1370e3
    def test_funct__rocket_ode__(self):
        # what is the structure of the rocket ode?
        # This is a basic test for getting outputs
        r0 = 6375.416e3
        theta = 0
        t = 0
        state = [r0*math.cos(theta), r0*math.sin(theta), 0, 0, 1]
        statedot = rocket_ode(t, state, self.mu_earth, 1, 1, lambda t, state: (0, 0))
        print(statedot)

    def test_funct__rocket_ode__1(self):
        ...
        # This test is a fall from 10 meters

if __name__ == '__main__':
    unittest.main()
