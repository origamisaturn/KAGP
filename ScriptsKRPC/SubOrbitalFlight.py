import krpc
import time

conn = krpc.connect(name='Sub-orbital Flight')
vessel = conn.space_center.active_vessel

vessel.auto_pilot.target_pitch_and_heading(80, 90)
vessel.auto_pilot.engage()
vessel.control.throttle = 1
time.sleep(1)

print('Launch!')
vessel.control.activate_next_stage()

ref_frame = vessel.orbit.body.reference_frame
flight = vessel.flight(ref_frame)
vert_vel = conn.get_call(getattr, flight, 'vertical_speed')
expr = conn.krpc.Expression.less_than(
    conn.krpc.Expression.call(vert_vel),
    conn.krpc.Expression.constant_double(0))
event = conn.krpc.add_event(expr)
with event.condition:
    event.wait()

print("Deploying Chutes")
vessel.control.activate_next_stage()