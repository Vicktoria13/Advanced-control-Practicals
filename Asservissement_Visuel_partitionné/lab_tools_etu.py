import numpy as np


def matrix_interaction(x, y, Z):
    """Image jacobian for a point visual feature"""
    '''A compléter'''

    # attention notation
    # le torseur cinématique se compose de 6 valeurs
    # vecteur vitesse V
    # vecteur vitesse angulaire W

    
    matrix = np.zeros((2, 6))
    matrix[0] = np.array([-1/Z, 0, x/Z, x*y, -(1+x**2), y])
    matrix[1] = np.array([0, -1/Z, y/Z, 1+y**2, -x*y, -x])

    return matrix
