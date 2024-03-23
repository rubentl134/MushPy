'''
This is example 2
It plots the trajectory of the particle at each iteration
The main objective is merge the plots in order to 
produce a video.
It produces many plots with name
fig1.png , fig2.png , fig3.png ...
'''
from magnetic import *
import numpy as np
import matplotlib.pyplot as plt
from stem2 import *
from libreria3 import *

gamma_left = -0.1
gamma_right = -0.1
base = 0.75
height = 1
B = 0.5#magnetic field
R = 1/B #radius

mushroom = mushroomPlotPoints(gamma_left,gamma_right,base,height)

	
theta = np.deg2rad(55)
vx,vy = -0.1,-0.01

x,y = np.cos(theta),np.sin(theta)

P = np.array([x,y])
v = np.array([vx,vy])

count = 0
plt.plot(mushroom[0],mushroom[1])
for i in range(20):
    
    out = magnetictilted(P,v,gamma_left,gamma_right,base,height,B)
    Pf = out[0]
    arco = plotArcTrajectory(P,Pf,v,B)
    plt.plot(arco[0],arco[1],color='g',linewidth=0.5)
    P,v = Pf,out[1]
    if P[1] > 0 :	
    	#VALIDANDO QUE SOLO SEA IMPACTO CON EL MUSHROOM HEAD
		
        theta = thetaAngle(P)		
        phi = phiAngle(theta,P)	
        alpha = alphaAngle(P,v,phi)	

    print(10*'-')
    ###
    #plt.show()
    count+=1
    f = 'fig'+str(count)+'.png' 
    print(f)
    plt.savefig(f)
    



