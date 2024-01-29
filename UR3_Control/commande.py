import math
from spatialmath import SE3, base
import numpy as np

# returns the error vector e, and the angular error theta
# inputs : T the current homogoneous transformation 0->T and Td : the target 0-->d
def compute_error(T, Td):
    
    # initializing an empty error array
    #les 3 premiers sont l'erreur de translation entre current et desired
    #les 3 derniers sont l'erreur de rotation exprimée dans axe angle

    e = np.empty(6)

    # Computing the error homogenous transformation martix expressed in the end-effector frame
    # T_T->Td = inv(T_0->T) * T_0->Td
    T_e = np.linalg.inv(T)@Td

    # Filling in the translation error expressed in the end-effector frame
    e[:3] = T_e[:3,-1]

    # Compution the error rotation matrix expressed in the end-effector frame
    R = T_e[:3, :3]


    # Compute the rotation axis expressed in the end-effector frame
    l = np.array([R[2, 1] - R[1, 2], 
                  R[0, 2] - R[2, 0], 
                  R[1, 0] - R[0, 1]])
    
    # Computing the rotation axis and angle
    # 1/Check if the vector in nill

    # cela signifie que l'on regarde si le vecteur l a des composantes 
    
    
    # CELA ARRIVE SI THETA = kPI (0 ou PI ?)
    #ainsi si cela arrive alors on a une rotation de 0 ou PI
    if base.iszerovec(l):

        #CAS OU THETA = 0
        if np.trace(R) > 0:
            # Complete with theta*u
            u = np.array([0, 0, 0])
            # Complete with theta
            theta = 0

        #CAS OU THETA = PI
        else:
            u = np.sqrt(np.diag(R)+1)/2
            theta = np.pi
    
    #si le vecteur n'est pas nul alors on a une rotation quelconque
    #il faut donc calculer theta et theta*u
            
    else:
        ln = np.linalg.norm(l)
        theta = np.arctan2(ln, (np.trace(R)-1))
        u = ((theta)/ln) * l

        
    e[3:] = u

    return e, theta

# returns the desired joints velocities(qd_tilde), the arrived flag, e and theta (vector and theta error)
# inputs : The robot instance, the current and the target transformations, the error accepted threshold and the current arrived flag

def cartesian_control(robot, T, Td, gain, threshold, arrived):

    # Computing the error 
    e, theta= compute_error(T, Td)
    
    # if the error is greater than the accepted threshold, we compute a control input
    if (np.sum(np.abs(e)) > threshold):
        # Compute the control input in the operational space
        #doit etre un vecteur de 6
        ee_wrench = gain*e
        qd_tilde = np.linalg.pinv(robot.jacobe(robot.q))@ee_wrench
    else:
        qd_tilde = np.array([0,0,0,0,0,0])
    arrived = True if np.sum(np.abs(e)) < threshold else False

    return qd_tilde, arrived, e, theta