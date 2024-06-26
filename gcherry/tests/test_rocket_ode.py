import unittest
import math
import numpy as np

from gcherry.rk4 import rk4
from gcherry.integration_interface import rocket_ode


class TestRocketOde(unittest.TestCase):
    def setUp(self):
        Isp = 300
        F_thrust_max = 100 #
        self.lunar_radius = 1740e3   # m
        self.lunar_mu = 4.9048695e12 # m^3/s^2
        self.mu_earth = 3.986004418e14
        self.earth_radius = 6378.1370e3

    def test_funct__rocket_ode__(self):
        # This is a basic test, tests that output of acceleration is equal
        # to standard gravity
        r0 = 6375.416e3
        theta = 0
        t = 0

        pos = [r0*math.cos(theta), r0*math.sin(theta), 0]
        vel = [0, 0, 0]
        m = [1]
        state = pos + vel + m
        guidance_func = lambda t, state: (0, 0, 0)
        statedot = rocket_ode(t, state, self.mu_earth, 1, 1, guidance_func)
        print(statedot)
        g0 = 9.80665
        a_mag = np.linalg.norm(statedot[3:6])

        self.assertTrue(abs(a_mag - g0) < 1e8)

    def test_funct__rocket_ode__1(self):
        # Tests a fall from a 10 meter height with arbitary global position
        # as dictated by theta.
        r0 = 6375.416e3
        init_height = r0 + 10
        theta = math.pi/4
        tspan = [0, 1.42809]
        max_step = 0.05

        pos = [init_height*math.cos(theta), init_height*math.sin(theta), 0]
        vel = [0, 0, 0]
        mass = [1]
        state = pos + vel + mass

        guidance_func = lambda t, state: (0, 0, 0)
        ode_func = lambda t, y: rocket_ode(t, y, self.mu_earth, 1, 1, guidance_func)
        t_res, y_res = rk4(ode_func, tspan, state, max_step)

        print(np.linalg.norm(y_res[0:2, -1]) - r0)
        r_mag = np.linalg.norm(y_res[0:3, -1])
        self.assertTrue(abs(r_mag - r0) < 1e-3)

    def test_funct__rocket_ode__2(self):
        # This tests an orbit about Earth and compares it to the analytic
        # solution

        r0 = 6375.416e3
        rp = 300e3 + r0
        ra = 400e3 + r0
        theta = 0

        va = 7641.53306186991
        pos = ra * np.array([math.cos(theta), math.sin(theta), 0])
        vel = va * np.array([-math.sin(theta), math.cos(theta), 0])
        mass = 1

        state = np.concatenate((pos, vel, [mass]))
        tspan = [0, 2744.4777483128987] 

        max_step = 10
        guidance_func = lambda t, state: (0, 0, 0)
        ode_func = lambda t, state: rocket_ode(t, state, self.mu_earth, 1, 1, guidance_func)
        t_res, y_res = rk4(ode_func, tspan, state, max_step)

        print("orbit res")
        r_mag = np.linalg.norm(y_res[0:2, -1])
        print(r_mag - rp)
        self.assertTrue(abs(r_mag - rp)<1e-2)

    def test_funct__rocket_ode__3(self):
        # Tests a fall from 10m along with thrusting horizontally, checks 
        # mass state before and after stopping thrusting
        # Warning, the thrust is not continuous at t=1 so rk4 needs a small
        # step size to handle it accurately.
        dv = 10
        T = 1
        m0 = 100
        ve = 2350
        F_thrust = 997.8753551745272

        def guidance_func(t, state):
            thrust_mag = 0
            thrust_pitch = 0
            thrust_yaw = np.deg2rad(90)
            if t >= 0 and t < 1:
                thrust_mag = 1
            return thrust_mag, thrust_pitch, thrust_yaw
        g0 = 9.80665
        Isp = ve/g0

        max_step = 0.01
        r0 = 6375.416e3
        init_height = r0 + 10
        # tspan0 is to test the fuel flow, tspan1 is to test final state
        # and fuel cutoff
        tspan0 = [0, 0.8]
        tspan1 = [0, 1.42809]
        pos = [init_height, 0, 0]
        vel = [0, 0, 0]
        mass = [m0]
        state = pos + vel + mass
        ode_func = lambda t, state: rocket_ode(t, state, self.mu_earth, Isp, F_thrust, guidance_func)
        t_res0, y_res0 = rk4(ode_func, tspan0, state, max_step)
        t_res1, y_res1 = rk4(ode_func, tspan1, state, max_step)

        mf0 = 99.66029775142995 
        mf1 = 99.57537218928744 

        mf0_res0 = y_res0[6, -1]
        mf1_res1 = y_res1[6, -1]

        # Calculated mass checks
        self.assertTrue(abs(mf0_res0 - mf0) < 1e-3)
        self.assertTrue(abs(mf1_res1 - mf1) < 1e-3)
        
        v = y_res1[4, -1]
        r_mag = np.linalg.norm(y_res1[0:3, -1])

        # Final position and velocity checks
        self.assertTrue(abs(r_mag - r0) < 1e-3)
        self.assertTrue(v - dv < 1e-3)

    def test_funct__rocket_ode__4(self):
        # Tests a suicide burn with landing at time T.
        m0 = 100
        ve = 2350
        F_thrust = 997.8753551745272
        def guidance_func(t, state):
            thrust_mag = 0
            thrust_pitch = 90 * math.pi/180
            thrust_yaw = np.deg2rad(90)
            if t >= -0.1 and t <= 3.1:
                thrust_mag = 1
            return thrust_mag, thrust_pitch, thrust_yaw
        g0 = 9.80665
        Isp = ve/g0
        r0 = 6375.416e3

        T = 3
        x0 = 6375417.159500072
        v0 = -0.7086221494046185
        xT = r0
        theta = np.deg2rad(30)

        pos = x0 * np.array([math.cos(theta), math.sin(theta), 0])
        vel = v0 * np.array([math.cos(theta), math.sin(theta), 0])
        mass = [m0]
        state = np.concatenate((pos, vel, mass))

        max_step = 0.1
        tspan = [0, 3]
        ode_func = lambda t, state: rocket_ode(t, state, self.mu_earth, Isp, F_thrust, guidance_func)
        t_res, y_res = rk4(ode_func, tspan, state, max_step)
        # sample_res = solve_ivp(ode_func, tspan, state, method='RK45')

        # Check against final radius and final velocity magnitudes
        res_r = y_res[0:3, -1]
        res_v = y_res[3:6, -1]
        res_r_mag = np.linalg.norm(res_r)
        res_v_mag = np.linalg.norm(res_v)
        print(res_r_mag)
        print(res_v_mag)
        self.assertTrue(abs(res_r_mag - xT)<1e-5)
        self.assertTrue(abs(res_v_mag - 0)<1e-5)
        print("final", y_res[:, -1])
        print("rmag: {}".format(res_r_mag))

    def test_funct__rocket_ode__5(self):
        ...


if __name__ == '__main__':
    unittest.main()
