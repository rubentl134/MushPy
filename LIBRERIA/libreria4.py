'''
This improves the code 
libreria3.py
please note that this file should replace libreria3.py
'''

'''
HAT MUSHROOM
This library let simulate the trajectory of a charged particle on the head of the mushroom.
'''

from magnetic import *
import numpy as np
import matplotlib.pyplot as plt
from stem3 import *

def mushroomPlotPoints(gamma_left,gamma_right,B,H):
    '''
    This function lets draw the mushroom and
    saves all points for draw tilted mushroom
    '''
    Pleft,Pright = hatLineIntersection(gamma_left,gamma_right,B)
    xMushroom,yMushroom = arcHatPlot(gamma_left,gamma_right,B)
    xplot = np.array([-B/2,-B/2,B/2,B/2,Pright[0]])
    yplot = np.array([0,-H,-H,0,Pright[1]])
    xMushroom = np.concatenate((xMushroom,xplot))
    yMushroom = np.concatenate((yMushroom,yplot))
    
    return [xMushroom,yMushroom]


def hatValido(hat,B,Pright,Pleft):
    '''
    Determines if a point belongs to the hat
    '''
    if hat[0]<-B/2: #punto lado izquierdo
        if hat[1] > Pleft[1]:
            #print('VALIDO')
            return hat		
        else:
            #print('NO VALIDO')
            return np.array([None,None])
    elif hat[0]>B/2:
        if hat[1] > Pright[1]:
            #print('Valido')
            return hat
        else:
            #print('No Valido')
            return np.array([None,None])

    else:
        #print('no valido?')
        if hat[1] > 0:
            #print('si valido')
            return hat
        else:
            #print('no valido')
            return np.array([None,None])


def magnetictilted(P,v,gamma_left,gamma_right,base,height,B):
	'''
	One movemment of the particle inside the mushroom's hat
	If particle get in the mushroom stem, the function return the entry coordinates
	and speed. Then the 'stem' library receives these arguments.
	'''
	
	Pleft,Pright = hatLineIntersection(gamma_left,gamma_right,base) #FLOOR AND HAT INTERSECTION
	slopeLeft = (Pleft[1]-0)/(Pleft[0]-(-base/2)) #LEFT FLOOR SLOPE
	slopeRight = (Pright[1]-0)/(Pright[0]-(base/2)) #RIGHT FLOOR SLOPE
	bLeft = slopeLeft*(base/2)
	bRight = -slopeRight*(base/2)
	wallRight = [slopeRight,bRight]
	wallLeft = [slopeLeft,bLeft]
	normalRightWall = unitary_vector(np.array([-Pright[1],Pright[0]-(base/2)]))
	normalLeftWall = unitary_vector(-np.array([-(Pleft[1]-0),Pleft[0]+(base/2)]))

    ##############
	A1 = P #guardando para plot
	centro = centroCirc(P,v,B)
	R = 1/B
	c = circunference(centro[0],centro[1],R)
    #plt.plot(c[0],c[1],'g')
    #ubicando el punto de partida
    #print('PUNTO DE PARTIDA')

    #unir centro a punto de partida
    #plt.plot([centro[0],P[0]],[centro[1],P[1]])
	hits = []

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
    
    
    #########################################################
    # AHORA VAMOS A HALLAR LA VELOCIDAD
    #########################################################

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
			exit = magneticStem(base,height,B,P,vIncidente)
            #print('exit',exit)
			P = exit[0]
			vReflejado = exit[1]
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

	return [P,v]

def plotArcTrajectory(start,end,v,B):
    '''
    Given the star and end of the circular trajectory, it returs 20 points
    of the trajectory.
    * INPUT
    - start: starting point of the trajectory
    - end: end point of the trajectory
    - v: velocity
    - B: magnetic field
    '''

    R = 1/B
    centro = centroCirc(start,v,B)
    c = circunference(centro[0],centro[1],R)
    A1 = start
    A2 = end

    #para A - start
    thetaA1 = thetaAngle(np.array([A1[0]-centro[0],A1[1]-centro[1]]))
    thetaA2 = thetaAngle(np.array([A2[0]-centro[0],A2[1]-centro[1]]))
    if thetaA1 < thetaA2:
        thetaList = np.linspace(thetaA1,thetaA2,num=20)
    else: #thetaA > thetaB
        thetaList = np.linspace(thetaA1,thetaA2+2*np.pi,num=20)  
    curveX = R*np.cos(thetaList)+centro[0]
    curveY = R*np.sin(thetaList)+centro[1]
    return [curveX,curveY]
