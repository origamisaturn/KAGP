import math

def final_time_unit_test():
    mu_earth = 3.986004418e14 # (m^3/s^2), gravitational parameter
    g0 = 9.80665 # m/s^2, standard gravity
    r0 = 6375.416e3 # m, radius at standard gravity

    x0 = r0 + 10
    xT = r0
    v0 = 0
    a = -g0

    t = (-v0 - math.sqrt(v0**2 - 2*a*(x0 - xT)))/a
    print("t = {}".format(t))


final_time_unit_test()
def half_orbit_unit_test():
    r0 = 6375.416e3
    rp = 300e3 + r0
    ra = 400e3 + r0
    mu = 3.986004418e14

    a = 1/2*(rp+ra)
    e = (ra - rp)/(ra + rp)
    h = math.sqrt((1-e**2)*a*mu)
    va = h/ra
    print(va)
    T = 2*math.pi/math.sqrt(mu) * a**(3/2)
    print("T/2: {}".format(T/2))

half_orbit_unit_test()

dv = 10
T = 1
m0 = 100
ve = 2350
mf = m0/math.exp(dv/ve)
F_thrust = (m0 - mf) * ve/T
mdot = F_thrust/ve
T2 = 0.8
mf2 = m0 - mdot*T2
print("F: {}".format(F_thrust))
print("mf: {}".format(mf))
print("mf2: {}".format(mf2))

