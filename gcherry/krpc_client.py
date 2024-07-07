import krpc
import numpy as np
from gcherry.guidance_interface import GuidanceBase
from gcherry.log import SimulationLog
from gcherry.config import Config


# TODO: Consider moving into same file as integrator sim.
class KRPCClient:
    guidance_obj: GuidanceBase
    log: SimulationLog
    _last_outer_loop_time: float
    _outer_loop_cutoff: float
    _outer_loop_interval: float

    def __init__(self, config: Config, guidance_obj: GuidanceBase):
        self.log = SimulationLog()
        self.guidance_obj = guidance_obj
        # TODO: These should be set by config.
        self._last_outer_loop_time = 0
        self._parse_input(config)
        self._connect()
        self._init_streams()

    def run(self):
        init_time = self._conn.space_center.ut
        # guidance must start at time 0
        guidance_time = 0
        state = self._get_state()
        # Initialize outer loop solution
        self.guidance_obj.get_command(
                        0, state, outer_loop=True, log=True)
        self._mark_outer_loop_calc(guidance_time)
        estimated_T = self.guidance_obj.estimated_final_time()

        # Default initial pitch and heading
        self._vessel.auto_pilot.attenuation_angle = (0.01, 0.01, 0.01)
        self._vessel.auto_pilot.target_pitch_and_heading(90, 90)
        self._vessel.auto_pilot.target_roll = 0
        self._vessel.auto_pilot.engage()
        self._vessel.control.throttle = 1
 
        while guidance_time < estimated_T:
            state = self._get_state()
            self._log_state(state, guidance_time)
            
            # Likely incorrect
            thrust_acc = self._get_thrust_acc()
            if (not self._is_outer_loop_cutoff(guidance_time) and 
                self._is_outer_loop_scheduled(guidance_time)):
                thrust_cmd, pitch_cmd, heading_cmd = (
                    self.guidance_obj.get_command(
                        guidance_time, state, outer_loop=True, log=True))
                self._mark_outer_loop_calc(guidance_time)
            else:
                thrust_cmd, pitch_cmd, heading_cmd = (
                    self.guidance_obj.get_command(
                        guidance_time, state, outer_loop=False, log=True))

            self._vessel.control.throttle = thrust_cmd
            self._vessel.auto_pilot.target_pitch_and_heading(
                np.rad2deg(pitch_cmd), np.rad2deg(heading_cmd))

            with self._streams['time'].condition:
                self._streams['time'].wait()

            guidance_time = self._streams['time']() - init_time
            estimated_T = self.guidance_obj.estimated_final_time()

            print("{:.2f}%\t{:.2f} deg\t{:.2f} deg\t{:.1f}s\t{:.1f}s".format(
                thrust_cmd*100,
                np.rad2deg(pitch_cmd),
                np.rad2deg(heading_cmd),
                guidance_time,
                estimated_T))
        
        self._vessel.control.throttle = 0
        self._vessel.auto_pilot.disengage()
        self.save_log()

    def _is_outer_loop_cutoff(self, t):
        """ True if less than outer_loop_cutoff seconds from guidance 
        termination. """
        T = self.guidance_obj.estimated_final_time()
        return (T - t) < self._outer_loop_cutoff
    
    def _is_outer_loop_scheduled(self, t):
        return (t - self._last_outer_loop_time) >= self._outer_loop_interval
    
    def _mark_outer_loop_calc(self, t):
        """ Sets last time outer loop was calculated. """
        self._last_outer_loop_time = t

    def _get_thrust_acc(self):
        return self._streams['thrust']()/self._streams['mass']()

    def _get_state(self):
        pos = self._streams['position']()
        vel = self._streams['velocity']()

        rhs_pos = ksp_to_rhs(pos)
        rhs_vel = ksp_to_rhs(vel)

        t = self._streams['time']()
        x = rhs_pos[0]
        y = rhs_pos[1]
        z = rhs_pos[2]
        vx = rhs_vel[0]
        vy = rhs_vel[1]
        vz = rhs_vel[2]
        m = self._streams['mass']()

        state = [x, y, z, vx, vy, vz, m]
        return state

    def _connect(self):
        """ Connect to local KRPC server. """
        self._conn = krpc.connect(name=self.client_name)
        self._vessel = self._conn.space_center.active_vessel
        
    def _init_streams(self):
        """ Initiate data streams from KRPC server. """
        conn = self._conn
        vessel = self._vessel

        self._streams = {}

        ref_frame = vessel.orbit.body.non_rotating_reference_frame
        self._streams['mass'] = conn.add_stream(getattr, vessel, 'mass')
        self._streams['position'] = conn.add_stream(vessel.position, ref_frame)
        self._streams['velocity'] = conn.add_stream(vessel.velocity, ref_frame)
        self._streams['time'] = conn.add_stream(getattr, conn.space_center, 'ut')   
        self._streams['thrust'] = conn.add_stream(getattr, vessel, 'thrust')

    def _parse_input(self, config):
        # self.client_name = input_dict['simulator']['name']
        self.client_name = "PLACEHOLDER"
        # self.sim_end_time = input_dict['simulator']['simulation_end_time']
        self._outer_loop_cutoff = config.krpc_client.outer_loop_cutoff
        self._outer_loop_interval = config.krpc_client.outer_loop_interval

    def _log_state(self, state, t):
        self.log.log_state(t, state)

def ksp_to_rhs(coord):
    # This assumes flight path along equator
    rot_mat = np.array([[1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])
    coord_rhs = rot_mat@coord
    return coord_rhs