def calculate_final_radial_state(a0, a1, a2, c1, c2, Tgo, r0, r_dot_0):
    f11 = a0*Tgo + a1*Tgo**2/2 + a2*Tgo**3/3
    f21 = a0*Tgo**2/2 + a1*Tgo**3/3 + a2*Tgo**4/4
    f12 = f21
    f22 = a0*Tgo**3/3 + a1*Tgo**4/4 + a2*Tgo**5/5

    r_dot_T = r_dot_0 + f11*c1 + f12*c2
    r_T = r0 + r_dot_0*Tgo + f21*c1 + f22*c2

    return r_T, r_dot_T

def calc_test_case_1():
    r0 = 1737.4e3
    r_dot_0 = 0
    Tgo = 438
    a0, a1, a2, c1, c2 = (5.182888241994683, 
                          -0.006887777058719676,
                          9.153481725927932e-06, 
                          -0.12670854928655143, 
                          0.0006085997594104416)
    r_T, r_dot_T = calculate_final_radial_state(a0, a1, a2, c1, c2, Tgo, r0, r_dot_0)
    return r_T, r_dot_T


if __name__ == '__main__':
    r_T, r_dot_T = calc_test_case_1()
    print("r_T: {}\nr_dot_T: {}".format(r_T, r_dot_T))
