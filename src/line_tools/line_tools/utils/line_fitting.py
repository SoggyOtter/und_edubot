import matplotlib.pyplot as plt
import numpy as np
from math import pi as pi

'''
This utility assume the formula of a line is 
L[0]*x+L[1]*y+L[3] = 0
'''

'''
Plots a line
'''
def plot_line(x,y,L, ax = None):
    xs, ys = proj_point_line(x[0],y[0],L)
    xf, yf = proj_point_line(x[-1],y[-1],L)
    if ax is None:
        plt.plot([xs, xf],[ys,yf], c='b', linestyle='-', marker='o')
    else:
        ax.plot([xs, xf],[ys,yf], c='b', linestyle='-', marker='o')
    #plt.plot(x,y, c='b', linestyle='-', marker='o')
    # plt.pause(0.1)

'''
Fits a line to an array of points 
'''
def line_fit(x, y) -> np.array:
   #Make the A matrix in Ax=0
   A = np.column_stack((x,y,np.ones(len(x))))
   
   #SVD of A
   U, s, Vh = np.linalg.svd(A)

   #Find the NULL vector
   min_idx = np.argmin(s)
   L = Vh[min_idx].T
   #Nomalize the line equation
   L = L/np.sqrt(L[0]**2+L[1]**2)

   return L
'''
Give the distance of a point or set of points
to the line L. Returns an array of distances.
'''
def dist_2_line(x, y, L) -> np.array:
    if isinstance(x, np.ndarray):
       dist = np.column_stack((x,y,np.ones(len(x)))) @ L
    else:
       dist = np.column_stack((x,y,1)) @ L

    return dist
'''
projects a point onto a line,
returns the projection 
'''
def proj_point_line(x,y,L):
   #points distance to the line
   D = dist_2_line(x,y,L)
   x_p = x - D*L[0]
   y_p = y - D*L[1]
   return x_p, y_p



'''
Takes a set of points and an equation for a line.
Returns the largest distance from a point is
from the line.
'''
def max_d_line(x,y,L)->float:
   dist = np.abs(dist_2_line(x, y, L))
   return np.max(dist)


'''
Rough length of a line
'''
def line_length(x,y,start,end):
    length = ((x[start]-x[end-1])**2 + (y[start]-y[end-1])**2)
    return np.sqrt(length)

'''
Find the biggest gap between any two points in the line
'''
def max_gap(x,y,start,end):
   gap = np.max( (x[start:end-1]-x[start+1:end])**2 +
                 (y[start:end-1]-y[start+1:end])**2)
   return np.sqrt(gap)

def max_gap_idx(x,y,start,end):
   return np.argmax((x[start:end-1]-x[start+1:end])**2 +
                 (y[start:end-1]-y[start+1:end])**2)