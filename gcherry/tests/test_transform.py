import unittest
import numpy as np
from gcherry.transform import (
    body2global_rot, global2body_rot,
    perifocal2global_rot, global2perifocal_rot,
    pcf2global_rot, global2pcf_rot,
    Rx
)

class TestRot(unittest.TestCase):
    def test_funct__body2global_rot__1(self):
        """ Tests:
        - body2global_rot()
            - get_ra_decl()
            - body2topo_rot()
            - topo2global_rot()
                - Rx
                - Ry
                - Rz
        """
        pos_global = [0.5, 0.5, np.sin(np.deg2rad(45))]
        roll = np.deg2rad(90)
        pitch = np.deg2rad(45)
        yaw = np.deg2rad(180)

        expected_global_axes = np.array([[1/2*np.sqrt(2), 0, -1/2*np.sqrt(2)],
                                         [1/2*np.sqrt(2), 0, 1/2*np.sqrt(2)],
                                         [0, -1, 0]])
        calculated_global_axes = body2global_rot(roll, pitch, yaw, pos_global)
        self.assertTrue(within_tol(expected_global_axes, calculated_global_axes, tol=1e-8))

        expected_body_axes = np.identity(3)
        calculated_body_axes = global2body_rot(roll, pitch, yaw, pos_global) @ calculated_global_axes
        self.assertTrue(within_tol(expected_body_axes, calculated_body_axes, tol=1e-8))

    def test_funct__body2global_rot__2(self):
        pos_global = [0.5*(3/2)**0.5, 0.5*(3/2)**0.5, np.sin(np.deg2rad(30))]
        roll = np.deg2rad(90)
        pitch = np.deg2rad(60)
        yaw = np.deg2rad(180)

        expected_global_axes = np.array([[1/2*np.sqrt(2), 0, -1/2*np.sqrt(2)],
                                         [1/2*np.sqrt(2), 0, 1/2*np.sqrt(2)],
                                         [0, -1, 0]])
        calculated_global_axes = body2global_rot(roll, pitch, yaw, pos_global)
        self.assertTrue(within_tol(expected_global_axes, calculated_global_axes, tol=1e-8))

        expected_body_axes = np.identity(3)
        calculated_body_axes = global2body_rot(roll, pitch, yaw, pos_global) @ calculated_global_axes
        self.assertTrue(within_tol(expected_body_axes, calculated_body_axes, tol=1e-8))

    def test_funct__perifocal2global_rot__(self):
        lan = np.deg2rad(30)
        inc = np.deg2rad(45)
        argp = np.deg2rad(90)

        expected_global_axes = np.array([[-0.35355339, -0.8660254,   0.35355339],
                                         [ 0.61237244, -0.5,        -0.61237244],
                                         [ 0.70710678,  0.,          0.70710678]])
        calculated_global_axes = perifocal2global_rot(lan, inc, argp)
        self.assertTrue(within_tol(expected_global_axes, calculated_global_axes, tol=1e-8))

        expected_body_axes = np.identity(3)
        calculated_body_axes = global2perifocal_rot(lan, inc, argp) @ calculated_global_axes
        self.assertTrue(within_tol(expected_body_axes, calculated_body_axes, tol=1e-8))

    def test_funct__pcf2global_rot__1(self):
        pos_global_1 = [1, 0, 0]
        pos_global_2 = [0, 0, 1]
        lan = np.deg2rad(0)
        inc = np.deg2rad(90)

        expected_global_axes_1 = np.array([[1, 0, 0],
                                         [0, 0, -1],
                                         [0, 1, 0]])
        calculated_global_axes_1 = pcf2global_rot(pos_global_1, lan, inc)
        self.assertTrue(within_tol(expected_global_axes_1, calculated_global_axes_1, tol=1e-8))
        expected_global_axes_2 = np.array([[0, -1, 0],
                                         [0, 0, -1],
                                         [1, 0, 0]])
        calculated_global_axes_2 = pcf2global_rot(pos_global_2, lan, inc)
        self.assertTrue(within_tol(expected_global_axes_2, calculated_global_axes_2, tol=1e-8))

        expected_body_axes_1 = np.identity(3)
        calculated_body_axes_1 = global2pcf_rot(pos_global_1, lan, inc) @ calculated_global_axes_1
        self.assertTrue(within_tol(expected_body_axes_1, calculated_body_axes_1, tol=1e-8))
        expected_body_axes_2 = np.identity(3)
        calculated_body_axes_2 = global2pcf_rot(pos_global_2, lan, inc) @ calculated_global_axes_2
        self.assertTrue(within_tol(expected_body_axes_2, calculated_body_axes_2, tol=1e-8))


    def test_funct___pcf2global_rot__2(self):
        pos_global_1 = [1, 0, 0]
        pos_global_2 = [0, 0, 1]
        lan = np.deg2rad(10)
        inc = np.deg2rad(90)

        expected_global_axes_1 = np.array([[1, 0, 0],
                                         [0, 0, -1],
                                         [0, 1, 0]])
        calculated_global_axes_1 = pcf2global_rot(pos_global_1, lan, inc)
        self.assertTrue(within_tol(expected_global_axes_1, calculated_global_axes_1, tol=1e-8))
        expected_global_axes_2 = np.array([[0, -1, 0],
                                         [0, 0, -1],
                                         [1, 0, 0]]) @ Rx(lan)
        calculated_global_axes_2 = pcf2global_rot(pos_global_2, lan, inc)
        self.assertTrue(within_tol(expected_global_axes_2, calculated_global_axes_2, tol=1e-8))

        expected_body_axes_1 = np.identity(3)
        calculated_body_axes_1 = global2pcf_rot(pos_global_1, lan, inc) @ calculated_global_axes_1
        self.assertTrue(within_tol(expected_body_axes_1, calculated_body_axes_1, tol=1e-8))
        expected_body_axes_2 = np.identity(3)
        calculated_body_axes_2 = global2pcf_rot(pos_global_2, lan, inc) @ calculated_global_axes_2
        self.assertTrue(within_tol(expected_body_axes_2, calculated_body_axes_2, tol=1e-8))


def within_tol(val1, val2, tol=1e-8):
    if np.all(abs(val1-val2) < tol):
        return True
    else:
        return False
    

if __name__ == '__main__':
    unittest.main()