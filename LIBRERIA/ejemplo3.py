'''
This is example 3
Generate the phase space data
Plot the phase space data.
Maybe: Implement a function to just generate the phase space
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
B = 0.05#magnetic field
R = 1/B #radius

theta = np.deg2rad(55)
#vx,vy = -0.1,-0.01

x,y = np.cos(theta),np.sin(theta)

P = np.array([x,y])


file1 = open('example3.dat',"w")

for j in [-0.01,-0.05,-0.1,-0.2,-0.3,-0.4,-0.5,-0.6]:
	v = np.array([-0.1,j])
	for i in range(500):
		out = magnetictilted(P,v,gamma_left,gamma_right,base,height,B)
		Pf = out[0]
		#print('Iteration: ',i)
		#print('Position:',Pf)
		#print('Theta angle: ',thetaAngle(Pf))
		arco = plotArcTrajectory(P,Pf,v,B)
		#plt.plot(arco[0],arco[1],color='g',linewidth=0.5)
		P,v = Pf,out[1]
		if P[1] > 0 :	
			#VALIDANDO QUE SOLO SEA IMPACTO CON EL MUSHROOM HEAD
			
		    theta = thetaAngle(P)		
		    phi = phiAngle(theta,P)	
		    alpha = alphaAngle(P,v,phi)	
		    file1.write(str(theta)+'\t'+str(alpha)+"\n")
		#print(10*'-')
		###

print('Check file example3.dat and plot it on gnuplot please.')

