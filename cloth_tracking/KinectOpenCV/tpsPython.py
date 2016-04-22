import numpy as np
from scipy.interpolate import Rbf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pylab as p

# Takes in a set of old control points and a set of new control points.
# The function then calculates the TPS interpolation for the Delta X, Delta Y, and Delta Z with respect to the X,Y for each pair of control points.
# The function returns 3 radial basis functions that can be used to calculate the dX, dY,dZ for any other point (x,y)   
def tpsWarp3d(XYZ_old, XYZ_new):

    XY_dx = np.concatenate((XYZ_old[:,[0]], XYZ_old[:,[1]], (XYZ_new[:,[0]] - XYZ_old[:,[0]])), axis=1)
    #print XY_dx    
    XY_dy = np.concatenate((XYZ_old[:,[0]], XYZ_old[:,[1]], (XYZ_new[:,[1]] - XYZ_old[:,[1]])), axis=1)
    XY_dz = np.concatenate((XYZ_old[:,[0]], XYZ_old[:,[1]], (XYZ_new[:,[2]] - XYZ_old[:,[2]])), axis=1)
    rbf_x = Rbf(XY_dx[:,0],XY_dx[:,1],XY_dx[:,2],function='thin-plate',smooth=0.0)
    rbf_y = Rbf(XY_dy[:,0],XY_dy[:,1],XY_dy[:,2],function='thin-plate',smooth=0.0)
    rbf_z = Rbf(XY_dz[:,0],XY_dz[:,1],XY_dz[:,2],function='thin-plate',smooth=0.0)
    return rbf_x, rbf_y, rbf_z
#Function takes in a set of old and new control points and uses that information to calculate the 3d TPS transform that will be applied to the provided mesh data points.
#The mesh data points are given in 3 mesh grids representing the X, Y, and Z values.
#The function returns a new set of 3 warped mesh grids with the same layout.
def warpPoints(XYZ_old, XYZ_new, X, Y, Z):
    rbf_x, rbf_y, rbf_z = tpsWarp3d(XYZ_old, XYZ_new)
    xi = rbf_x(X, Y) + X
    yi = rbf_y(X, Y) + Y
    zi = rbf_z(X, Y) + Z
    XI = np.array([xi]).T
    YI = np.array([yi]).T
    ZI = np.array([zi]).T
    #return np.concatenate((XI, YI, ZI), axis=1)
    return xi, yi, zi
#Function that needs to be called to set up the plotting system. (NO LONGER NEEDED)
def init():
    global fig, fig2, subplot, subplot2
    fig = plt.figure()
    fig2 = plt.figure()
    subplot = fig.add_subplot(1,1,1, projection='3d', adjustable='box', aspect=1)
    subplot2 = fig2.add_subplot(1,1,1, projection='3d', adjustable='box', aspect=1)
    
    
    return 1
#Function that can be called from an external program and will display the 3d tps warped mesh that the corresponding data would create.
#The intent of this is that the function could be modified to return the data for use as an expectation that the cloth data could be checked against. 
def displayWarp(XYZ_old, XYZ_new, X_m, Y_m, Z_m):
    global fig, subplot
    print "Warping"
    #print XYZ_old
    subplot.clear()
    XYZ = np.array(XYZ_old)
    XYZ2 = np.array(XYZ_new)
    #plotTPS(XYZ, 15, subplot)
    GRID_POINTS = 8
    x_min = XYZ[:,0].min()
    x_max = XYZ[:,0].max()
    y_min = XYZ[:,1].min()
    y_max = XYZ[:,1].max()
    z_min = XYZ[:,2].min()
    z_max = XYZ[:,2].max()
    xi = np.linspace(x_min, x_max, GRID_POINTS)
    yi = np.linspace(y_min, y_max, GRID_POINTS)
    zi = np.linspace(z_min, z_max, GRID_POINTS)
    g, ZI= np.meshgrid(xi, zi)
    XI, YI, = np.meshgrid(xi, yi)
    #print XI
   # X,Y,Z = warpPoints(XYZ, XYZ2, XI,YI,ZI)
    X,Y,Z = warpPoints(XYZ, XYZ2, X_m, Y_m, Z_m)
    subplot.plot_wireframe(X, Y, Z)

    subplot.scatter(XYZ[:,0],XYZ[:,1],XYZ[:,2], 'z', 40, 'r', True)
    subplot.scatter(XYZ2[:,0],XYZ2[:,1],XYZ2[:,2], 'z', 40, 'g', True)

    plt.xlabel('X')
    plt.ylabel('Y')
    fig.canvas.draw()
    plt.show(block=False)
    return 1


def display2DWarpWithDepth(X_m, Y_m, Z_m):
    global fig2, subplot2
    #fig = plt.figure()
    
    subplot2.clear()
    #subplot2.scatter(X_m,Y_m,Z_m, 'z', 40, 'g', True)
    subplot2.plot_wireframe(X_m, Y_m, Z_m)
    
    subplot2.set_title('Graphs of 2d TPS with Raw Depth Data')
    subplot2.set_xlabel('X in Pixels')
    subplot2.set_ylabel('Y in Pixels')
    subplot2.set_zlabel('Z in mm')
    subplot2.set_xlim(50, 500)    
    subplot2.set_ylim(100, 700)
    subplot2.set_zlim(400, 1800)
    subplot2.set_aspect(1)
    
    fig2.canvas.draw()
    plt.show(block=False)
    return 1
'''    
init()
x = np.array([[0,1,2],
              [0,1,2],
              [0,1,2]])
y = np.array([[0,0,0],
              [1,1,1],
              [2,2,2]])
z = np.array([[0,1,0],
              [0,0,0],
              [0,2,0]])
display2DWarpWithDepth(x, y, z)
'''
