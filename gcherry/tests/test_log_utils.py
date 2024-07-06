import unittest
import numpy as np
import math

from gcherry.log_utils import (
    get_radius,
    get_acc, 
    get_r_dot, 
    get_v_theta, 
    get_r_dot_dot, 
    get_a_theta)


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

    def test_funct__get_ground_distance__(self):
        ...

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
                        [1, 0, -1]])
        t = [0, 0.1, 0.2]
        acc = get_acc(t, vel)
        mid_acc = acc[:, 1]
        expected_mid_acc = np.array([15, -10])
        error = np.linalg.norm(mid_acc - expected_mid_acc)
        self.assertTrue(error < 1e-8)

if __name__ == '__main__':
    unittest.main()