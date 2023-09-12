import math

mu_earth = 3.986004418e14 # (m^3/s^2), gravitational parameter
g0 = 9.80665 # m/s^2, standard gravity
r0 = 6375.416e3 # m, radius at standard gravity

x0 = r0 + 10
xT = r0
v0 = 0
a = -g0

t = (-v0 - math.sqrt(v0**2 - 2*a*(x0 - xT)))/a
print("t = {}".format(t))

