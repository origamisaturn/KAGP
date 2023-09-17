import unittest
from scipy.integrate import solve_ivp
from cherryInt import rocket_ode
import math
import numpy as np

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
        state = [r0*math.cos(theta), r0*math.sin(theta), 0, 0, 1]
        statedot = rocket_ode(t, state, self.mu_earth, 1, 1, lambda t, state: (0, 0))
        print(statedot)
        g0 = 9.80665
        a_mag = np.linalg.norm(statedot[2:4])

        self.assertTrue(abs(a_mag - g0) < 1e8)

    def test_funct__rocket_ode__1(self):
        # Tests a fall from a 10 meter height with arbitary global position
        # as dictated by theta.
        r0 = 6375.416e3
        init_height = r0 + 10
        theta = math.pi/4
        tspan = [0, 1.42809]
        state = [init_height*math.cos(theta), init_height*math.sin(theta), 0, 0, 1]
        res = solve_ivp(rocket_ode, tspan, state, args=(self.mu_earth, 1, 1, lambda t, state: (0, 0)))
        print(np.linalg.norm(res.y[0:2, -1]) - r0)
        r_mag = np.linalg.norm(res.y[0:2, -1])

        self.assertTrue(abs(r_mag - r0) < 1e-3)

    def test_funct__rocket_ode__2(self):
        # This tests an orbit about Earth and compares it to the analytic
        # solution

        r0 = 6375.416e3
        rp = 300e3 + r0
        ra = 400e3 + r0
        theta = 0

        va = 7641.53306186991
        r = ra * np.array([math.cos(theta), math.sin(theta)])
        v = va * np.array([-math.sin(theta), math.cos(theta)])
        m = 1

        state = np.concatenate((r, v, [m]))
        tspan = [0, 2744.4777483128987] 

        # Learn more about rtol and atol, I set these arbitrarily
        res = solve_ivp(rocket_ode, tspan, state, args=(self.mu_earth, 1, 1, lambda t, state: (0, 0)), atol=1e-9, rtol = 1e-9)

        print("orbit res")
        r_mag = np.linalg.norm(res.y[0:2, -1])
        print(r_mag - rp)
        self.assertTrue(abs(r_mag - rp)<1e-2)

    def test_funct__rocket_ode__3(self):
        # Tests a fall from 10m along with thrusting horizontally, checks 
        # mass state before and after stopping thrusting
        dv = 10
        T = 1
        m0 = 100
        ve = 2350
        F_thrust = 997.8753551745272

        def guidance_func(t, state):
            thrust_mag = 0
            thrust_angle = 0
            if t >= 0 and t < 1:
                thrust_mag = 1
            return thrust_mag, thrust_angle
        g0 = 9.80665
        Isp = ve/g0

        r0 = 6375.416e3
        init_height = r0 + 10
        # tspan0 is to test the fuel flow, tspan1 is to test final state
        # and fuel cutoff
        tspan0 = [0, 0.8]
        tspan1 = [0, 1.42809]
        state = [init_height, 0, 0, 0, m0]
        res0 = solve_ivp(rocket_ode, tspan0, state, args=(self.mu_earth, Isp, F_thrust, guidance_func), atol=1e-9, rtol = 1e-9)
        res1 = solve_ivp(rocket_ode, tspan1, state, args=(self.mu_earth, Isp, F_thrust, guidance_func), atol=1e-9, rtol = 1e-9)

        mf0 = 99.66029775142995 
        mf1 = 99.57537218928744 

        mf0_res0 = res0.y[4, -1]
        mf1_res1 = res1.y[4, -1]

        self.assertTrue(abs(mf0_res0 - mf0) < 1e-3)
        self.assertTrue(abs(mf1_res1 - mf1) < 1e-3)
        
        v = res1.y[3, -1]
        r_mag = np.linalg.norm(res1.y[0:2, -1])

        self.assertTrue(abs(r_mag - r0) < 1e-3)
        self.assertTrue(v - dv< 1e-3)

    def test_funct__rocket_ode__4(self):
        # Tests a suicide burn with landing at time T.
        m0 = 100
        ve = 2350
        F_thrust = 997.8753551745272
        def guidance_func(t, state):
            thrust_mag = 0
            thrust_angle = 90 * math.pi/180
            if t >= 0 and t <= 3:
                thrust_mag = 1
            return thrust_mag, thrust_angle
        g0 = 9.80665
        Isp = ve/g0
        r0 = 6375.416e3

        T = 3
        x0 = 6375417.159500072
        v0 = -0.7086221494046185
        xT = r0
        theta = -30 * math.pi/180

        r = x0 * np.array([math.cos(theta), math.sin(theta)])
        v = v0 * np.array([math.cos(theta), math.sin(theta)])
        state = np.concatenate((r, v ,[m0]))

        tspan = [0, 3]
        res = solve_ivp(rocket_ode, tspan, state, args=(self.mu_earth, Isp, F_thrust, guidance_func), atol=1e-9, rtol = 1e-9)

        # Check against final radius and final velocity magnitudes
        res_r = res.y[0:2, -1]
        res_v = res.y[2:4, -1]
        res_r_mag = np.linalg.norm(res_r)
        res_v_mag = np.linalg.norm(res_v)
        self.assertTrue(abs(res_r_mag - xT)<1e-5)
        self.assertTrue(abs(res_v_mag - 0)<1e-5)
        print("final", res.y[:, -1])
        print("rmag: {}".format(res_r_mag))

if __name__ == '__main__':
    unittest.main()
