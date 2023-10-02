import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt

def col_mult(mat1, mat2):
    samples = mat1.shape[1]
    mult = np.zeros(samples)
    for i in range(samples):
        mult[i] = np.dot(mat1[:, i], mat2[:, i])
    return mult

def get_radius(log):
    pos = np.array([log['state']['x'], log['state']['y']])
    radius = np.linalg.norm(pos, axis=0)
    return radius

def get_ground_distance(log, ref_pos):
    # ref_pos determines ground radius and origin angle
    radius_ref = np.linalg.norm(ref_pos)
    angle_ref = np.arctan2(ref_pos[1], ref_pos[0])
    distance_ref = radius_ref * angle_ref

def get_r_hat(log):
    pos = np.array([log['state']['x'], log['state']['y']])
    r_hat = pos/np.linalg.norm(pos, axis=0)
    return r_hat

def get_theta_hat(log):
    r_hat = get_r_hat(log)
    rot_mat = np.array([[0, -1], [1, 0]])
    theta_hat = rot_mat@r_hat
    return theta_hat

def get_r_dot(log):
    v = np.array([log['state']['vx'], log['state']['vy']])
    r_hat = get_r_hat(log)
    r_dot = col_mult(r_hat, v)
    return r_dot

def get_v_theta(log):
    v = np.array([log['state']['vx'], log['state']['vy']])
    pos = np.array([log['state']['x'], log['state']['y']])
    theta_hat = get_theta_hat(log)
    v_theta = col_mult(theta_hat, v)
    return v_theta

def get_r_dot_dot(log):
    acc = get_acc(log)
    r_hat = get_r_hat(log)
    r_dot_dot = col_mult(acc, r_hat)
    return r_dot_dot

def get_a_theta(log):
    acc = get_acc(log)
    theta_hat = get_theta_hat(log)
    a_theta = col_mult(acc, theta_hat)
    return a_theta

def get_acc(log):
    vx = log['state']['vx']
    vy = log['state']['vy']
    t = log['state']['t']
    v = np.array((vx, vy))
    acc = np.gradient(v, t, axis=1)
    return acc

def get_orbital_elements(log):
    ...

def get_alpha(log):
    # the input and output already have alpha
    ...

if __name__ == '__main__':    
    log_file = "log.pkl"
    with open(log_file, 'rb') as fh:
        log = pkl.load(fh)
    #print(log)
    r = get_radius(log)
    #y = get_r_dot_dot(log)
    y = get_r_dot(log)
    origin = [log['state']['x'][0], log['state']['y'][0]]
    s = get_ground_distance(log, origin)
    t = log['state']['t']
    t_samp = log['inputs']['pitch_query.t']
    r_dot_dot_pred = log['outputs']['pitch_query.r_dot_dot']
    plt.plot(t, y)
    plt.figure()
    plt.plot(t_samp, r_dot_dot_pred)
    plt.show()
