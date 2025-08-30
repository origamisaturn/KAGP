import unittest
import math
import numpy as np

from kagp.log_utils import (
    get_radius,
    get_r_hat,
    get_theta_hat,
    get_acc,
    get_r_dot,
    get_v_theta,
    get_r_dot_dot,
    get_a_theta,
    get_gravity,
#    get_non_gravity_acc,
#    get_non_gravity_acc_mag,
    get_orbital_elements,
#    get_thrust_pitch,
    get_projected_true_anomaly,
#    get_thrust_acc_PCF,
#    get_theta_hat_PCF,
    get_target_normal_position,
    get_target_normal_velocity,
    get_target_normal_acceleration,
    get_time_steps,
    almost_equal)


class TestConversions(unittest.TestCase):
    def setUp(self):
        self.log = {'state': {}}

    def test_funct__get_radius__(self):
        theta = 30 * math.pi/180
        pos = np.array([[np.sin(theta), 10],
                        [np.cos(theta), 20],
                        [0, 0]])
        radius = get_radius(pos)
        expected_radius = np.array([1, math.sqrt(10**2 + 20**2)])
        error = np.linalg.norm(radius - expected_radius)
        self.assertTrue(error < 1e-8)

    def test_funct__r_hat__(self):
        theta = 30 * math.pi/180
        pos = np.array([[np.sin(theta), 10],
                        [np.cos(theta), 20],
                        [0, 0]])
        r_hat = get_r_hat(pos)
        expected_r_hat = np.array([[np.sin(theta), 0.4472136],
                                   [np.cos(theta), 0.89442719],
                                   [0, 0]])
        error = np.linalg.norm(r_hat - expected_r_hat)
        self.assertTrue(error < 1e-6)

    def test_funct__theta_hat__(self):
        theta = 30 * math.pi/180
        pos = np.array([[np.sin(theta), 10],
                        [np.cos(theta), 20],
                        [0, 0]])
        vel = np.array([[-1, -1],
                        [0, 0],
                        [0, 0]])
        theta_hat = get_theta_hat(pos, vel)
        expected_theta_hat = np.array([[-np.cos(theta), -0.89442719],
                                   [np.sin(theta), 0.4472136],
                                   [0, 0]])
        error = np.linalg.norm(theta_hat - expected_theta_hat)
        self.assertTrue(error < 1e-6)

    def test_funct__get_r_dot__(self):
        r0 = 1737e3
        pos = np.array([[r0, 2*r0/(2)**0.5, 0],
                        [0, 2*r0/(2)**0.5, r0],
                        [0, 0, 0]])
        vel = np.array([[5, 5, 5],
                        [0, 0, 0],
                        [0, 0, 0]])
        r_dot = get_r_dot(pos, vel)
        r_dot_expected = np.array([5, 5/(2)**0.5, 0])
        error = np.linalg.norm(r_dot - r_dot_expected)
        self.assertTrue(error < 1e-8)

    def test_funct__get_v_theta__(self):
        r0 = 1737e3
        pos = np.array([[r0, 2*r0/(2)**0.5, 0],
                        [0, 2*r0/(2)**0.5, r0],
                        [0, 0, 0]])
        vel = np.array([[5, 5, 5],
                        [0, 0, 0],
                        [0, 0, 0]])
        v_theta = get_v_theta(pos, vel)
        v_theta_expected = np.array([0, 5/(2)**0.5, 5])
        error = np.linalg.norm(v_theta - v_theta_expected)
        self.assertTrue(error < 1e-8)

    def test_funct__get_r_dot_dot__(self):
        r0 = 1737e3
        pos = np.array([r0/(2)**0.5 * np.ones(3),
                        r0/(2)**0.5 * np.ones(3),
                        0 * np.ones(3)])
        vel = np.array([[5, 10, 20],
                        [0, 0, 0],
                        [0, 1, 2]])
        t = np.array([0, 1, 2])
        r_dot_dot = get_r_dot_dot(t, pos, vel)
        r_dot_dot_expected = np.array([5, 7.5, 10])/(2)**0.5
        error = np.linalg.norm(r_dot_dot - r_dot_dot_expected)
        self.assertTrue(error < 1e-8)

    def test_funct__get_a_theta__(self):
        r0 = 1737e3
        pos = np.array([r0/(2)**0.5 * np.ones(3),
                        r0/(2)**0.5 * np.ones(3),
                        0 * np.ones(3)])
        vel = np.array([[5, 10, 20],
                        [0, 0, 0],
                        [0, 0, 0]])
        t = np.array([0, 1, 2])
        a_theta = get_a_theta(t, pos, vel)
        a_theta_expected = np.array([5, 7.5, 10])/(2)**0.5
        error = np.linalg.norm(a_theta - a_theta_expected)
        self.assertTrue(error < 1e-8)

    def test_funct__get_acc__(self):
        vel = np.array([[3, 5, 6],
                        [1, 0, -1],
                        [0, 0, 0]])
        t = [0, 0.1, 0.2]
        acc = get_acc(t, vel)
        mid_acc = acc[:, 1]
        expected_mid_acc = np.array([15, -10, 0])
        error = np.linalg.norm(mid_acc - expected_mid_acc)
        self.assertTrue(error < 1e-8)

    def test_funct__get_gravity__(self):
        mu = 4.9028e+12
        r0 = 1737e3
        pos = np.array([[r0, r0/(2)**0.5, 0],
                        [0, r0/(2)**0.5, r0],
                        [0, 0, 0]])
        gravity = get_gravity(pos, mu)
        gravity_expected = np.array([[1, 1/(2)**0.5, 0],
                                    [0, 1/(2)**0.5, 1],
                                    [0, 0, 0]]) * -1.62496698
        error = np.linalg.norm(gravity - gravity_expected)
        self.assertTrue(error < 1e-8)

    # def test_funct__get_non_gravity_acc__(self):
    #     self.assertTrue(False)

    # def test_funct__get_non_gravity_acc_mag__(self):
    #     self.assertTrue(False)

    def test_funct__get_orbital_elements__(self):
        # lunar radius
        r0 = 1737.4e3
        mu = 4.9028e12

        # Test for a 50km x 50km altitude orbit about the Moon
        vp = 1656.1940188803374
        pos = np.array([[0], [50e3 + r0], [0]])
        vel = np.array([[-vp*np.cos(np.deg2rad(60))], [0], [vp*np.sin(np.deg2rad(60))]])

        oe = get_orbital_elements(pos, vel, mu)
        a, e, i, lan, argp, nu = tuple(oe.T[0])
        self.assertTrue(almost_equal(a, 50e3 + r0)
                        and almost_equal(e, 0)
                        and almost_equal(i, np.deg2rad(60))
                        and almost_equal(lan, np.deg2rad(90))
                        and almost_equal(argp, 0)
                        and almost_equal(nu, 0))

        # Test for a 50km x 100km altitude orbit about the Moon
        vp = 1667.5775555183266
        pos = np.array([[50e3 + r0], [0], [0]])
        vel = np.array([[0], [vp*np.cos(np.deg2rad(10))], [vp*np.sin(np.deg2rad(10))]])

        oe = get_orbital_elements(pos, vel, mu)
        a, e, i, lan, argp, nu = tuple(oe.T[0])
        self.assertTrue(almost_equal(a, 75e3 + r0)
                        and almost_equal(e, 0.01379386448907527)
                        and almost_equal(i, np.deg2rad(10))
                        and almost_equal(lan, 0)
                        and almost_equal(argp, 0)
                        and almost_equal(nu, 0))

    # def test_funct__get_thrust_pitch__(self):
    #     self.assertTrue(False)

    def test_funct__get_projected_true_anomaly__(self):
        pos = np.array([[0], [1], [0]])
        target_lan = np.array([np.deg2rad(90)])
        target_inc = np.array([np.deg2rad(45)])
        target_argp = np.array([np.deg2rad(90)])
        nu_proj = get_projected_true_anomaly(
            pos, target_lan, target_inc, target_argp)[0]
        self.assertTrue(almost_equal(nu_proj, np.deg2rad(-90)))

    # def test_funct__get_thrust_acc_PCF__(self):
    #     self.assertTrue(False)

    # def test_funct__get_theta_hat_PCF__(self):
    #     self.assertTrue(False)

    def test_funct__get_target_normal_position__(self):
        target_lan = np.deg2rad(-90)
        target_inc = np.deg2rad(60)
        pos = np.array([[1], [0], [0]])
        normal_pos = get_target_normal_position(pos, target_lan, target_inc)[0]
        self.assertTrue(almost_equal(normal_pos, -np.sin(np.deg2rad(60))))

    def test_funct__get_target_normal_velocity__(self):
        target_lan = np.deg2rad(-90)
        target_inc = np.deg2rad(60)
        vel = np.array([[1], [0], [0]])
        normal_vel = get_target_normal_velocity(vel, target_lan, target_inc)[0]
        self.assertTrue(almost_equal(normal_vel, -np.sin(np.deg2rad(60))))

    def test_funct__get_target_normal_acceleration__(self):
        target_lan = np.deg2rad(-90)
        target_inc = np.deg2rad(60)
        vel = np.array([[5, 10, 20],
                        [0, 0, 0],
                        [0, 0, 0]])
        t = np.array([0, 1, 2])
        normal_acc = get_target_normal_acceleration(t, vel, target_lan, target_inc)
        expected_normal_acc = np.array([5, 7.5, 10]) * -np.sin(np.deg2rad(60))
        self.assertTrue(almost_equal(normal_acc, expected_normal_acc))

    def test_funct__get_time_steps__(self):
        t = [0, 0.1, 0.2, 0.5, 1]
        dt = get_time_steps(t)
        self.assertTrue(almost_equal(dt, [0, 0.1, 0.1, 0.3, 0.5]))


if __name__ == '__main__':
    unittest.main()
