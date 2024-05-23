import numpy as np

def Rx(angle: float):
    c1 = np.cos(angle)
    s1 = np.sin(angle)
    return np.array([[1, 0, 0], 
                     [0, c1, -s1],
                     [0, s1, c1]])

def Ry(angle: float):
    c1 = np.cos(angle)
    s1 = np.sin(angle)
    return np.array([[c1, 0, s1], 
                     [0, 1, 0],
                     [-s1, 0, c1]])

def Rz(angle: float):
    c1 = np.cos(angle)
    s1 = np.sin(angle)
    return np.array([[c1, -s1, 0], 
                     [s1, c1, 0],
                     [0, 0, 1]])

def unit_vector(vec):
    return vec/np.linalg.norm(vec)

""" Body: Frame fixed to vehicle CoM, rotates with the vehicle. X is 
        forward, Y to the right, Z down.
    Topocentric: Frame origin at vehicle CoM, axes direction dependent 
        on global location. X is North, Y is East, Z is toward the global origin.
    Global: Frame origin at center of celestial body. Is inertial.
        X is RA 0 decl 0, Y is RA 90 decl 0, Z is decl 90.
    Perifocal: Frame origin at center of celestial body. X points to 
        periapsis, Y is true anomaly 90 degrees in orbital plane, Z is
        normal to orbital plane.
    Radial-Circumferential-Normal: Frame origin at vehicle CoM, X is 
        radial, Y points to the local horizon toward the direction
        of vehicle travel, Z is normal, collinear with angular
        momentum vector.
    Plane Control Frame: Frame origin at vehicle CoM, X is radial, Y is
        along the cross of the normal vector of desired orbital plane and X,
        Z points to the local horizon toward the direction of the normal
        vector of the desired orbital plane.

    """

def perifocal2global_rot(lan, inc, argp):
    """ Rotation from perifocal to global axes. 
    Args:
        lan: [rad.] Longitude of Ascending Node
        inc: [rad.] Inclination
        argp: [rad.] Argument of Periapsis
        
    """
    return Rz(lan)@Rx(inc)@Rz(argp)

def global2perifocal_rot(lan, inc, argp):
    """ Rotation from global to perifocal axes.
    See perifocal2global_rot().
    
    """
    return perifocal2global_rot(lan, inc, argp).T

def pcf2global_rot(pos_global, lan, inc):
    """ Rotation from plane control frame to global axes. 
    Args:
        pos_global: [m] 3-element vector, position in global frame.
        lan: [rad.] Longitude of Ascending Node
        inc: [rad.] Inclination
        
    """
    argp = 0 # argp does not affect orbit normal vector.
    orbit_normal_global = perifocal2global_rot(lan, inc, argp)@np.array([0, 0, 1])
    x = unit_vector(pos_global)
    y = unit_vector(np.cross(orbit_normal_global, x))
    z = unit_vector(np.cross(x, y))
    return np.stack((x, y, z), axis=-1)

def global2pcf_rot(pos_global, lan, inc):
    """ Rotation from global axes to plane control axes. 
    See pcf2global_rot().
    
    """
    return pcf2global_rot(pos_global, lan, inc).T

def get_ra_decl(pos_global):
    """ Gets right ascension and declination of given position. 
    Args:
        pos_global: [m] 3-element vector, position in global frame.

    """
    x, y, z = tuple(pos_global)
    ra = np.arctan2(y, x)
    decl = np.arctan2(z, np.linalg.norm([x, y]))
    return ra, decl

def body2topo_rot(roll, pitch, yaw):
    """ Rotation from body to topocentric axes. 
    Args:
        roll: [rad.]
        pitch: [rad.]
        yaw: [rad.]

    """
    return Rz(yaw)@Ry(pitch)@Rx(roll)

def topo2body_rot(roll, pitch, yaw):
    """ Rotation from topocentric to body axes. 
    See body2topo_rot().
    
    """
    return body2topo_rot(roll, pitch, yaw).T

# TODO: Change input to be pos_global instead
def topo2global_rot(ra, decl):
    """ Rotation from topocentric to global axes. 
    Args:
        ra: [rad.] Right Ascension
        decl: [rad.] Declination
        
    """
    axes_switch_rot = np.array([[0, 0, -1],
                                [0, 1, 0],
                                [1, 0, 0]])
    # latitude negated as positive y rot is downwards
    return Rz(ra)@Ry(-decl)@axes_switch_rot

def global2topo_rot(ra, decl):
    """ Rotation from global to topocentric axes. 
    See topo2global_rot().
    
    """
    return topo2global_rot(ra, decl).T

def body2global_rot(roll, pitch, yaw, pos_global):
    """ Rotation from body to global axes. 
    Args:
        roll: [rad.]
        pitch: [rad.]
        yaw: [rad.]
        pos_global: [m] 3-element vector, position in global frame.
        
    """
    ra, decl = get_ra_decl(pos_global)
    return topo2global_rot(ra, decl) @ body2topo_rot(roll, pitch, yaw)

def global2body_rot(roll, pitch, yaw, pos_global):
    """ Rotation from global to body axes.
    See global2body_rot().
        
    """
    return body2global_rot(roll, pitch, yaw, pos_global).T

def global2rcn_rot(pos_global, vel_global):
    return rcn2global_rot(pos_global, vel_global).T

def rcn2global_rot(pos_global, vel_global):
    raise NotImplementedError()
