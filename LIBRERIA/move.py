'''
Just one move for the particle. 
This must be numerical. No plot at all
'''

from magnetic import *
import numpy as np
import matplotlib.pyplot as plt
from stem3 import *
from libreria4 import *

gamma_left = -0.1
gamma_right = -0.1
base = 0.75
height = 1
B = 0.05#magnetic field
R = 1/B #radius

#Create the geometry of the table

Pleft,Pright = hatLineIntersection(gamma_left,gamma_right,base) #FLOOR AND HAT INTERSECTION
slopeLeft = (Pleft[1]-0)/(Pleft[0]-(-base/2)) #LEFT FLOOR SLOPE
slopeRight = (Pright[1]-0)/(Pright[0]-(base/2)) #RIGHT FLOOR SLOPE
bLeft = slopeLeft*(base/2)
bRight = -slopeRight*(base/2)
wallRight = [slopeRight,bRight]
wallLeft = [slopeLeft,bLeft]
normalRightWall = unitary_vector(np.array([-Pright[1],Pright[0]-(base/2)]))
normalLeftWall = unitary_vector(-np.array([-(Pleft[1]-0),Pleft[0]+(base/2)]))

print('TABLE GEOMETRY')
print('P_left: ',Pleft,'P_right: ',Pright) 
print('Slope left wall: ',slopeLeft) 
print('Slope right wall: ',slopeRight) 
print('b left wall: ',bLeft )
print('b right wall: ',bRight )
print('Right wall equation: ',wallRight) 
print('Left wall equation: ',wallLeft )
print('Right wall normal: ',normalRightWall) 
print('Left wall normal: ',normalLeftWall )

theta = np.deg2rad(55)
#vx,vy = -0.1,-0.01

x,y = np.cos(theta),np.sin(theta)

P = np.array([x,y])
vx,vy = -0.1,-0.1
v = np.array([vx,vy])

vStem = v

centro = centroCirc(P,v,B) #coordenadas del centro de circunferencia
hits = [] #lista para guardar posibles impactos



# VERIFICAR IMPACTOS

'''
calculator section. remove this
'''
inicial = P

###########################################
#VERIFICAR LAS INTERSECCIONES CON EL HAT
###########################################
	
hat1,hat2 = hatCircleIntersection(centro,R)
#print('INTERSECCION HAT',hat1,hat2)
if hat1[0] == None:
    #no hay intersecciones
	hat1 = np.array([None,None])
	hat2 = np.array([None,None])
else:
    #hay intersecciones y por lo menos una es valida
	hat1 = hatValido(hat1,base,Pleft,Pright)
	hat2 = hatValido(hat2,base,Pleft,Pright)

 #print('HAT ',hat1,hat2)
if hat1[0] != None:
	hits.append(hat1)
if hat2[0] != None:
	hits.append(hat2)

 #print('INTERSECCION HAT',hat1,hat2)
    
 #################################################
 #INTERSECCION CON LA PARED IZQUIERDA
 #################################################
    
izq1,izq2 = wallCircleIntersection(wallLeft,centro,R)
if izq1[0] == None:
	pass
     #izq1,izq2=None
else:
	if izq1[0] < Pleft[0] or izq1[0]>-base/2:
		izq1 = np.array([None,None])
	if izq2[0] < Pleft[0] or izq2[0]>-base/2:
		izq2 = np.array([None,None])
 #print('LEFT WALL: ',izq1,izq2)
if izq1[0] != None:
	hits.append(izq1)
if izq2[0] != None:
	hits.append(izq2)
    
####################################################    
#INTERSECCION CON LA PARED DERECHA
####################################################
    
der1,der2 = wallCircleIntersection(wallRight,centro,R)
if der1[0] == None:
	pass
     #izq1,izq2=None
else:
	if der1[0] > Pright[0] or der1[0] < base/2:
		der1 = np.array([None,None])
	if der2[0] > Pright[0] or der2[0] < base/2:
		der2 = np.array([None,None])
 #print('RIGHT WALL: ',der1,der2)
if der1[0] != None:
	hits.append(der1)
if der2[0] != None:
	hits.append(der2)
        
###########################################################
 #INTERSECCION CON LA BASE    
 ###########################################################
base1,base2 = intersection_base(centro,R)

if base1[0] == None:
	pass
else:
	if base1[0] < -base/2 or base1[0] > base/2:
		base1 = np.array([None,None])
	if base2[0] < -base/2 or base2[0] > base/2:
		base2 = np.array([None,None])

 #print('horizontal: ', base1,base2)
if base1[0] != None:
	hits.append(base1)
if base2[0] != None:
	hits.append(base2)
 #list of hits
 #print(10*'*')
 #print('estos son los hits: ',hits)
 #angles
punto_partida = thetaAngle(np.array([P[0]-centro[0],P[1]-centro[1]]))
 #print('partida',np.rad2deg(punto_partida))
points_list = []
resta_list = []
 #hallar el punto de partida
for point in hits:
	resta = np.abs(punto_partida - thetaAngle(np.array([point[0]-centro[0],point[1]-centro[1]])))
     #print('resta',point,np.rad2deg(resta))
	if resta < 10e-8 :
         #print('partida',point)
		pass
	else:
		points_list.append(point)
		resta_list.append(resta)
#print('final lista',points_list)
###hallar angulo respecto al punto de partida

vPartida = np.array([P[0]-centro[0],P[1]-centro[1]]) 
#print('vector partida',vPartida)
anguloPoint_list = []
points_list2 = [] 
for hit in points_list:
       #print(5*'*')
	vhit = np.array([	hit[0]-centro[0],hit[1]-centro[1]	])
	cos = (vPartida[0]*vhit[0] + vPartida[1]*vhit[1])/(modulo(vPartida)*modulo(vhit))
	arccos = np.arccos(cos)
	cruz = np.cross([vPartida[0],vPartida[1],0],[vhit[0],vhit[1],0])
       #print('vvhit',vhit,'arccos',arccos,'cruz',cruz[2])
	if cruz[2] < 0:
           #mayor de 180
		if cos > 0:
			anguloPoint = 2*np.pi - np.abs(np.arccos(cos))
		else:
			anguloPoint = 2*np.pi - np.abs(np.arccos(cos)) 
	else:
           #menor de 180
		anguloPoint = np.arccos(cos)
		if anguloPoint < 0:
			anguloPoint = np.pi + anguloPoint
        #eliminando si se pasa algun angulo = 0.0 , osea el pto de partida
	if anguloPoint < 10e-8:
           #print('revisa actualiacion')
		pass
	else:
		points_list2.append(hit)
		anguloPoint_list.append(anguloPoint)
        #print(hit,'angulo',np.rad2deg(anguloPoint))	
   #print('point list',points_list)
   #print('point list update',points_list2)
   #print('lista angulos para puntos',np.rad2deg(anguloPoint_list))
   #print('index choque',np.argmin(anguloPoint_list))
choque = points_list2[np.argmin(anguloPoint_list)]
   #print('choque:',i,choque)
hit = choque
P = choque


print('HIT:',P)

#########################################################
# AHORA VAMOS A HALLAR LA VELOCIDAD
########################################################

#print(10*'-','velocidad')


if P[0] < -base/2:
     #LADO IZQUIERDO
     #print('lado izq')
	if P[0] < Pleft[0]:
           #HAT LADO IZQUIERDO
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)		
		vReflejado = reflexVelocityHat(hit,vIncidente)
	else:
           #PISO INCLINADO IZQUIERDO or hat
           #print('PISO INCLINADO IZQUIERDO or hat')
		if P[1] > 0:
               #hat izquierdo
			d = hit - centro
			vIncidente = rotar(d,np.pi/2)		
			vReflejado = reflexVelocityHat(hit,vIncidente)

		elif P[1] < 0:
               #piso inclinado izquierdo
               #print('piso inclinado izquierdo')
			d = hit - centro
			vIncidente = rotar(d,np.pi/2)			
			vReflejado = reflexVelocityWall(normalLeftWall,vIncidente)


elif P[0] > -base/2 and P[0] < base/2:
       #ZONA CENTRAL
	if P[1] == 0 :
           # BASE HORIZONTAL
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)
		vReflejado = np.array([	vIncidente[0],-vIncidente[1]])
           #print('PISO!!',P,vIncidente)
           #plt.arrow(P[0],P[1],vIncidente[0],vIncidente[1])
		print('***ingreso al stem***')           
		P,vReflejado =stemOneMove(base,height,B,P,vStem)
		
		
		
		#exit = magneticStem(base,height,B,P,vIncidente)
           #print('exit',exit)
		#P = exit[0]
		#vReflejado = exit[1]
           #print('salida',P)
           #print('velo salida',v)
           #plt.plot(P[0],P[1],'kX')
           #plt.arrow(P[0],P[1],v[0],v[1])
           #plt.show()
           #break		
	else:
           #HAT ZONA CENTRAL
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)		
		vReflejado = reflexVelocityHat(hit,vIncidente)

elif P[0] > base/2:

        #print('lado derecho')
        #LADO DERECHO
	if P[0] > Pright[0]:
           #HAT LADO DERECHO
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)		
		vReflejado = reflexVelocityHat(hit,vIncidente)
	else:
           #print('piso inclinado o hat')
           #PISO INCLINADO derecho or hat
           ###
           #INCLUIMOS una seecion cuado gamma derecha mas de 0 o negativo
           #
           #
           ###
		if gamma_right > 0:
               #print('gamma right > 0')
               #modificamos esto para gamma mayor q 0
			if P[1] > Pright[1]:
                   #hat derecho
                   #print('hat derecho')
				d = hit - centro
				vIncidente = rotar(d,np.pi/2)		
				vReflejado = reflexVelocityHat(hit,vIncidente)

			elif P[1] < Pright[1]:
                   #piso inclinado derecha
                   #print('piso inclinado derecha***')
				d = hit - centro
				vIncidente = rotar(d,np.pi/2)			
				vReflejado = reflexVelocityWall(normalRightWall,vIncidente)

		else:
               #print('gamma right < 0')
               #dejamos esto sin modificar
			if P[1] > 0:
                   #hat derecho
                   #print('hat derecho')
				d = hit - centro
				vIncidente = rotar(d,np.pi/2)		
				vReflejado = reflexVelocityHat(hit,vIncidente)

			elif P[1] < 0:
                   #piso inclinado izquierdo
                   #print('piso inclinado izquierda***')
				d = hit - centro
				vIncidente = rotar(d,np.pi/2)			
				vReflejado = reflexVelocityWall(normalRightWall,vIncidente)



else:
       #problemas problemas
	print('hay poblemas')

   ###
   # FIN HALLAR VELOCIDAD
   ###
   #plt.arrow(hit[0],hit[1],vIncidente[0],vIncidente[1])
   #plt.arrow(hit[0],hit[1],vReflejado[0],vReflejado[1])
v = vReflejado

#if P[1] == 0:
	# INGRESO al stem
#	print('***ingreso al stem***')
#	print(stemOneMove(base,height,B,P,vStem))
#	P,v =stemOneMove(base,height,B,P,vStem)
print('VELOCITY REFLEX:',v)


'''
Just CALCULATOR SECTION. THIS WILL BE REMOVED
'''
final = P
plt.plot(inicial[0],inicial[1],'ro')
plt.plot(final[0],final[1],'ro')
mushroom = mushroomPlotPoints(gamma_left,gamma_right,base,height)
plt.plot(mushroom[0],mushroom[1])
plt.show()

