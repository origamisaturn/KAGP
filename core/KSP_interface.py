import krpc
import numpy as np


class KSP2DInterface:
    def __init__(self, input_dict, guidance_interface, log_interface):
        self.log_interface = log_interface
        self.guidance_interface = guidance_interface
        self._parse_input(input_dict)
        self._init_log()
        self._connect()
        self._init_streams()
        ...
    def run(self):
        init_time = self._conn.space_center.ut
        guidance_time = self._streams['time']() - init_time

        self._vessel.auto_pilot.target_pitch_and_heading(90, 90)
        self._vessel.auto_pilot.target_roll = 0
        self._vessel.auto_pilot.engage()
        self._vessel.control.throttle = 1

        # Implement this
        #estimated_T = ...
        default_heading = 90 #deg
        while guidance_time < self.sim_end_time:
            state = self._get_state()
            self._log_state(state, guidance_time)
            
            # Likely incorrect
            thrust_acc = self._get_thrust_acc()
            command = self.guidance_interface.get_command(state, guidance_time, logging=True, thrust_acc=thrust_acc)
            thrust, alpha = tuple(command)
            print("{:.2f}%\t{:.2f} deg\t{:.1f}s".format(thrust*100,
                                                        np.rad2deg(alpha),
                                                        guidance_time))
            self._vessel.control.throttle = thrust
            self._vessel.auto_pilot.target_pitch_and_heading(np.rad2deg(alpha), default_heading)

            with self._streams['time'].condition:
                self._streams['time'].wait()

            guidance_time = self._streams['time']() - init_time
            # Implement this
            # estimated_T = self.guidance_interface.get_predicted_final_time()
        
        self._vessel.control.throttle = 0
        self._vessel.auto_pilot.disengage()

        self.save_log()

    def _get_thrust_acc(self):
        return self._streams['thrust']()/self._streams['mass']()

    def _get_state(self):
        pos = self._streams['position']()
        vel = self._streams['velocity']()

        rhs_pos_2d = ksp_to_rhs_2d(pos)
        rhs_vel_2d = ksp_to_rhs_2d(vel)

        t = self._streams['time']()
        x = rhs_pos_2d[0]
        y = rhs_pos_2d[1]
        vx = rhs_vel_2d[0]
        vy = rhs_vel_2d[1]
        m = self._streams['mass']()

        state = [x, y, vx, vy, m]
        return state

    def _log_state(self, state, t):
        # TODO implement this
        self.log_interface.log_state(state, t)

    def _connect(self):
        self._conn = krpc.connect(name=self.client_name)
        self._vessel = self._conn.space_center.active_vessel
        
    def _init_streams(self):
        conn = self._conn
        vessel = self._vessel

        self._streams = {}

        ref_frame = vessel.orbit.body.non_rotating_reference_frame
        self._streams['mass'] = conn.add_stream(getattr, vessel, 'mass')
        self._streams['position'] = conn.add_stream(vessel.position, ref_frame)
        self._streams['velocity'] = conn.add_stream(vessel.velocity, ref_frame)
        self._streams['time'] = conn.add_stream(getattr, conn.space_center, 'ut')   
        self._streams['thrust'] = conn.add_stream(getattr, vessel, 'thrust')

    def _parse_input(self, input_dict):
        self.client_name = input_dict['simulator']['name']
        self.sim_end_time = input_dict['simulator']['simulation_end_time']
        ...
    def _init_log(self):
        self.log_interface.init_sim_log()

    def _log_res(self):
        ...

    def save_log(self):
        self.log_interface.save()

def ksp_to_rhs_2d(coord):
    # This assumes flight path along equator
    rot_mat = np.array([[1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])
    coord_rhs = rot_mat@coord
    coord_rhs_2d = coord_rhs[:2]
    return coord_rhs_2d