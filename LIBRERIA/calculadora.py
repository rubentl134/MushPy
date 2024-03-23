from magnetic import *
import numpy as np
import matplotlib.pyplot as plt
from stem2 import *
from libreria3 import *
####
# CALCULATIONS
####


gamma_left = -0.05
gamma_right = -0.5
base = 0.75
height = 1
B = 0.5#magnitc field
R = 1/B #radius

mushroom = mushroomPlotPoints(gamma_left,gamma_right,base,height)
plt.plot(mushroom[0],mushroom[1])

theta1 = np.deg2rad(55)
x1,y1 = np.cos(theta1),np.sin(theta1)
vx1,vy1 = -0.1,-0.1
P1 = np.array([x1,y1])
v1 = np.array([vx1,vy1])

theta2 = np.deg2rad(55)
x2,y2 = np.cos(theta2),np.sin(theta2)
vx2,vy2 = -1,-1.49
P2 = np.array([x2,y2])
v2 = np.array([vx2,vy2])


for i in range(10):
    out1 = magnetictilted(P1,v1,gamma_left,gamma_right,base,height,B)
    Pf1 = out1[0]
    arco1 = plotArcTrajectory(P1,Pf1,v1,B)
    plt.plot(arco1[0],arco1[1],color='g',linewidth=0.5)
    P1,v1 = Pf1,out1[1]
    ###
    #out2 = magnetictilted(P2,v2,gamma_left,gamma_right,base,height,B)
    #Pf2 = out2[0]
    #arco2 = plotArcTrajectory(P2,Pf2,v2,B)
    #plt.plot(arco2[0],arco2[1],color='b',linewidth=1)
    #P2,v2 = Pf2,out2[1]


plt.show()
