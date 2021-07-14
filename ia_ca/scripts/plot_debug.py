from math import sin, cos, atan2, pi
import matplotlib.cm as cmx
import matplotlib.colors as colors

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.transforms import Affine2D
import numpy as np


def plot_sec(a,b,h,k,r_inner,r_outer, clr="b"):
        if (b<a):
            b = b+2*pi
        
        t = np.linspace(a,b)
        x1 = [(r_outer*cos(i))+h for i in t]
        y1 = [(r_outer*sin(i))+k for i in t]
        x2 = [(r_inner*cos(i))+h for i in t]
        y2 = [(r_inner*sin(i))+k for i in t]    
        x = x1 + x2[::-1]+ [x1[0]]
        y = y1 + y2[::-1] + [y1[0]]

        x = np.array(x).T
        y = np.array(y).T
        xy = [x, y]
        xy = np.array(xy).T
        
        fov= Polygon(xy, closed=True, color=clr, alpha=0.5)
        return fov

def getXbot(xb):
    '''
        Reshuffling the indices of xb to be compatile for plotting
    '''
    indices = [2,3,4,5,0,1]
    temp = [0 for i in xb]

    jdx = 0
    for idx in indices:
        temp[jdx] = xb[idx]
        jdx = jdx + 1

    return temp

def get_cc_angular_diff(start, end):
    diff_angle = 0.0

    if end*start > 0: ## both have same sign
            diff_angle = end - start
    else:
        if end < 0:
            diff_angle = 2*pi - start + end
        else:
            diff_angle = end - start
    return diff_angle



fig, ax = plt.subplots(1,1, figsize=(8,8))

Wm, Vm, Ts = 0.3, 0.15, 2
x1, y1, theta1 = 0,0,0


start_angle1 = theta1 - Wm*Ts
if(start_angle1 < -pi):
    start_angle1 = start_angle1 + 2*pi
    
end_angle1 = theta1 + Wm*Ts
if (end_angle1 > pi):
    end_angle1 = end_angle1 - 2*pi

sa = atan2(sin(theta1 - Wm*Ts), cos(theta1 - Wm*Ts))
ea = atan2(sin(theta1 + Wm*Ts), cos(theta1 + Wm*Ts))

print("Difference sA - sa  = {}, sE - se = {}".format(start_angle1-sa, end_angle1-ea))

Af = [  0, Vm*Ts, start_angle1, end_angle1, x1, y1]
Af_reordered = getXbot(Af)
print("{} ".format(Af_reordered))


a = plot_sec(*Af_reordered)
b = plot_sec(*Af_reordered)
#t = Affine2D().rotate(-1.57).translate(0.25, -0.25) + ax.transData


ax.add_artist(a)
#a.set_transform(t)

#b.update_from(a)

# ax.add_artist(a)
#ax.add_artist(b)


t = a.get_transform()
tt = Affine2D().translate(0, -0.25) + t
a.set_transform(tt) 

ax.set_xlim([-2,2])
ax.set_ylim([-2,2])
ax.grid()
plt.show()