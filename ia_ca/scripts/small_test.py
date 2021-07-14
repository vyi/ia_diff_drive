
import numpy as np
from math import pi

def is_higher(A, B):
    '''
    Accepts two angles A, B
    Both need to be within [-pi, pi]
    A and B represent a sector of a circle with center O, with one arm OA (start) and other OB (end)

    Because of jump of theta from -pi to pi, there is an ambiguous case are OA and arm OB 
    located on different sides (i.e. one angle is positive and other is negative)

    Returns
    -------
    True : When OA - OB makes CW sector [hypothesis]
    False : When OA - OB makes CCW sector [alternate hypothesis]

    '''
    if (A*B < 0) and (A <0):    ## Handles case: when both A and B have different sign and A<0
        return True
    else:
        if ((A - B)>0):
            return True
        else:
            return False

### Compute the mid angle for a CC sector
def get_cc_mid_angle(start, end):

    if end*start < 0: 
        if end < 0:
            end = 2*pi + end

    mid_angle = 0.5*(end + start)
    return mid_angle
    

ma_angle, robot_heading= get_cc_mid_angle(2.888, -3.000), 2.6825
print("mid angle for {} and {} is {}".format(2.888,-3.000, get_cc_mid_angle(2.888, -3.000)))
print(is_higher(ma_angle, robot_heading))