import pickle
import matplotlib.pyplot as plt
import os
import numpy as np
import math as m
from scipy.ndimage import uniform_filter1d

# Tranf function
def Rx(theta):
    theta = m.radians(theta)
    return np.matrix([[1, 0, 0],
                      [0, m.cos(theta), -m.sin(theta)],
                      [0, m.sin(theta), m.cos(theta)]])
def Ry(theta):
    theta = m.radians(theta)
    return np.matrix([[m.cos(theta), 0, m.sin(theta)],
                      [0, 1, 0],
                      [-m.sin(theta), 0, m.cos(theta)]])
def Rz(theta):
    theta = m.radians(theta)
    return np.matrix([[m.cos(theta), -m.sin(theta), 0],
                      [m.sin(theta), m.cos(theta), 0],
                      [0, 0, 1]])

# def Force_transf(dx,dy,dz,force):
#     # input: dx,dy,dz in degrees
#     #       force in list[x,y,z] or [0,0.1,0]
#     # output: transformed force in list [x,y,z,0,0,0]
#
#     R = np.round(Rx(dx)*Ry(dy)*Rz(dz),decimals=3)
#
#     R = R.transpose()
#     R[:,2] = -R[:,2]
#     print R
#     input_force = np.array(force)
#     print input_force
#     output_force = np.matmul(R,input_force)
#     print output_force
#     output_force = [output_force[0],output_force[1],output_force[2],0,0,0]#str(output_force[])
#
#     return output_force


def ConstraintForce_tranf(dx,dy,dz,ConstraintForce):
    # input: dx,dy,dz in degrees
    #       force in np.array([100,3])
    # output: transformed force in list [x,y,z,0,0,0]

    R = np.round(Rx(dx)*Ry(dy)*Rz(dz),decimals=3)

    R = -R.transpose()
    R[:,2] = -R[:,2]
    print R
    #input_force = np.array(force)
    #print input_force
    shape = ConstraintForce.shape
    #print shape
    for i in range(shape[0]): # for i in 100
        ConstraintForce[i,:] = np.matmul(R,ConstraintForce[i,:])
    #output_force = np.matmul(R,ConstraintForce)
    #print ConstraintForce.shape
    #output_force = [output_force[0],output_force[1],output_force[2]]#str(output_force[])

    return ConstraintForce



#global definitions
angle = 15  # in degree
mode = "tetra"
if mode == "grid":
    file_name = os.getcwd() + '\exps\grid\data_' + str(angle) + '.pickle' #global file dir
elif mode == "tetra":
    file_name = os.getcwd() + '\\exps\\tetra\\data_' + str(angle) + '.pickle'  # global file dir
elif mode == "trian":
    file_name = os.getcwd() + '\\exps\\trian\\data_' + str(angle) + '.pickle'
step_size = 0.001*5
window_size = 1


# Extract the data
# b should be list in form [[x,y,z],[x,y,z]]
with open(file_name, 'rb') as handle:
    b = pickle.load(handle)

len = len(b)
print len


# Process the data
xpoints = np.arange(0,len*step_size,step_size)
ypoints_global = np.array(b)
ypoints_local = ConstraintForce_tranf(270,-angle,90,ypoints_global)
ypoints_x = ypoints_local[:,0]
ypoints_y = ypoints_local[:,1]
ypoints_z = ypoints_local[:,2]
ypoints_sum = np.linalg.norm(ypoints_local,2,1)
#print ypoints_sum.shape
#print ypoints.shape

# mean over a sliding windows

ypoints_sum_x = uniform_filter1d(ypoints_x, size=window_size)
ypoints_sum_y = uniform_filter1d(ypoints_y, size=window_size)
ypoints_sum_window = uniform_filter1d(ypoints_sum, size=window_size)
#max_value = np.max(ypoints_sum_window) # to normalized it
max_value = 12569.21/17
mask = (ypoints_sum_window/max_value*22)>40
ypoints_sum_window[mask]=0
ypoints_sum_x[mask]=0
ypoints_sum_y[mask]=0


# Viz in a graph
plt.plot(xpoints, ypoints_sum_x/max_value*22,label="F_x")
plt.plot(xpoints, ypoints_sum_y/max_value*22,label="F_y")
plt.plot(xpoints, ypoints_sum_window/max_value*22,label="F_total")
plt.legend()
plt.ylabel('collision force/mN')
plt.xlabel('time/s')
plt.title(mode+'_'+str(angle))
plt.show()


# point1 = [0.070721, 0, 0]
# point2 = [0.311173, 0, 0]
# point1 = Force_transf(270, -1 * angle, 90, point1)
# point2 = Force_transf(270, -1 * angle, 90, point2)
# print point1
# print point2
