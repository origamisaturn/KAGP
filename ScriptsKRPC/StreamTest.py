import krpc
import time
import math
import numpy as np

def ksp_to_rhs_2d(coord):
    rot_mat = np.array([[1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])
    coord_rhs = rot_mat@coord
    coord_rhs_2d = coord_rhs[:2]
    return coord_rhs_2d

conn = krpc.connect(name='Sub-orbital Flight')
vessel = conn.space_center.active_vessel

ref_frame = vessel.orbit.body.non_rotating_reference_frame
mass = conn.add_stream(getattr, vessel, 'mass')
position = conn.add_stream(vessel.position, ref_frame)
velocity = conn.add_stream(vessel.velocity, ref_frame)
time = conn.add_stream(getattr, conn.space_center, 'ut')  

# while True:
#     #print(position())
#     #print(velocity())
#     with time.condition:
#         time.wait()
#     print(time())
#     print(position())
#     print(ksp_to_rhs_2d(position()))
