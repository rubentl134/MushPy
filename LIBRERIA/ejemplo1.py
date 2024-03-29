from magnetic import *
import numpy as np
import matplotlib.pyplot as plt
from stem2 import *
from libreria3 import *
####
# This is example 1
# Given the initial condicitions, the program prints the position
# and velocity at each iteration.
# Also, saves the phase space data as a .dat file
# so one can plot it.
# FIX: INITIAL DATA SHOHULD BE ALPHA ANGLE, NOT COORDINATES
# FIX: PRINT HAT COLLISIONS 
####



gamma_left = -0.1
gamma_right = -0.1
base = 0.75
height = 1
B = 0.05#magnetic field
R = 1/B #radius


mushroom = mushroomPlotPoints(gamma_left,gamma_right,base,height)
plt.plot(mushroom[0],mushroom[1])
	

theta = np.deg2rad(55)
vx,vy = -0.1,-0.01

x,y = np.cos(theta),np.sin(theta)

P = np.array([x,y])
v = np.array([vx,vy])

print('*** WELCOME TO MAGNETIC MUSHROOM BILLIARDS ***')
print('Initial theta angle:',theta)
print('Initial velocity angle:',v)
print('Magnetic field:',B)
file1 = open('data.dat',"w")

for i in range(1000):
    out = magnetictilted(P,v,gamma_left,gamma_right,base,height,B)
    Pf = out[0]
    #print('Iteration: ',i)
    #print('Position:',Pf)
    #print('Theta angle: ',thetaAngle(Pf))
    arco = plotArcTrajectory(P,Pf,v,B)
    plt.plot(arco[0],arco[1],color='g',linewidth=0.5)
    P,v = Pf,out[1]
    if P[1] > 0 :	
    	#VALIDANDO QUE SOLO SEA IMPACTO CON EL MUSHROOM HEAD
		
        theta = thetaAngle(P)		
        phi = phiAngle(theta,P)	
        alpha = alphaAngle(P,v,phi)	
        #print('ITERACION',i)
        #print(np.rad2deg(theta),np.rad2deg(alpha))
		#file1.write(str(theta)+'\t'+str(alpha)+"\n")
        file1.write(str(theta)+'\t'+str(alpha)+"\n")
    #print(10*'-')
    ###

    
print('END PROGRAM')

plt.show()z
