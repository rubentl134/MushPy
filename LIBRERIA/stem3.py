'''
This function improves stem2.py
this just should make one iteration on the stem
just numerical results. no plotting pleas
'''

import numpy as np
from magnetic import *


def stemOneMove(base,altura,B,P,v):
	'''
	One movement on the stem
	* INPU: base, altura, B, P, v
	* OUTPUT: P,v
	'''
	R = 1/B #radius
	centro = centroCirc(P,v,B)
	hits = []
	
	########
	#	HORIZONTAL SUPERIOR
	########
	
	if R**2 - centro[1]**2 >= 0:
		#	VERIFICAR CHOQUE CON TECHO
		#	horizontal superior
		#print('choque techo')
		xa,xb = np.sqrt(R**2 - centro[1]**2)+centro[0],-np.sqrt(R**2 - centro[1]**2)+centro[0]
		if xa < xb:
			x1 = xa
			x2 = xb
		else:
			x1 = xb
			x2 = xa
			
		if x1 < -base/2 or x1 > base/2:
			#x1 = None
			hs1 = np.array([None,None])
		else:
			hs1 = np.array([x1,0.0])		
			hits.append(hs1)
		if x2 < -base/2 or x2 > base/2:
			#x2 = None
			hs2 = np.array([None,None])
		else:
			hs2 = np.array([x2,0.0])
			hits.append(hs2)
			#print(hs1,hs2)
	
	########
	#	VERTICAL IZQUIERDA
	########
	
	if R**2 - (-base/2 - centro[0])**2 >= 0:
		#	VERIFICAR CHOQUE PARED IZQUIERDA
		#	vertical left
		#print('choque pared izquierda')
		ya,yb = np.sqrt(R**2-(-base/2 - centro[0])**2)+centro[1],-np.sqrt(R**2 - (-base/2 - centro[0])**2)+centro[1]				
		if ya > yb:
			y1 = ya
			y2 = yb
		else:
			y1 = yb
			y2 = ya
			
		if y1 > 0 or y1 < -altura:
			vl1 = np.array([None,None])
		else:
			vl1 = np.array([-base/2,y1])
			hits.append(vl1)
		if y2 > 0 or y2 < -altura:
			vl2 = np.array([None,None])
		else:
			vl2 = np.array([-base/2,y2])
			hits.append(vl2)
			#print(vl1,vl2)

	########
	#	HORIZONTAL INFERIOR
	########
	if R**2 - (-altura - centro[1])**2 >= 0:
		#	VERIFICAR CHOQUE CON LA BASE
		#	horizontal inferior
		#print('choque base')
		xa,xb = np.sqrt(R**2 - (-altura-centro[1])**2)+centro[0], -np.sqrt(R**2 - (-altura-centro[1])**2)+centro[0]
		if xa < xb:
			x1 = xa
			x2 = xb
		else:
			x1 = xb
			x2 = xa
			
		if x1 < -base/2 or x1 > base/2:
			#x1 = None
			hi1 = np.array([None,None])
		else:
			hi1 = np.array([x1,-altura])
			hits.append(hi1)		
		if x2 < -base/2 or x2 > base/2:
			#x2 = None
			hi2 = np.array([None,None])
		else:
			hi2 = np.array([x2,-altura])
			hits.append(hi2)
		#print(hi1,hi2)	
		
	
	########
	#	VERTICAL DERECHA
	########
	
	if R**2 - (base/2 - centro[0])**2 >= 0:
		#	VERIFICAR CHOQUE PARED DERECHA
		#	vertical right
		#print('choque pared derecha')
		ya,yb=np.sqrt(R**2 - (base/2 - centro[0])**2) + centro[1],-np.sqrt(R**2 - (base/2-centro[0])**2) + centro[1]
		if ya > yb:
			y1 = ya
			y2 = yb
		else:
			y1 = yb
			y2 = ya
				
		if y1 > 0 or y1 < -altura:
			vr1 = np.array([None,None])
		else:
			vr1 = np.array([base/2,y1])
			hits.append(vr1)
		if y2 > 0 or y2 < -altura:
			vr2 = np.array([None,None])
		else:
			vr2 = np.array([base/2,y2])
			hits.append(vr2)
		#print(vr1,vr2)

		#print('hits-->',hits)
	punto_partida = thetaAngle(np.array([P[0]-centro[0],P[1]-centro[1]]))
	#print('partida',np.rad2deg(punto_partida))
	points_list = []
	resta_list = []
	#hallar el punto de partida
	for point in hits:
		resta = np.abs(punto_partida - thetaAngle(np.array([point[0]-centro[0],point[1]-centro[1]])))
		#print(point,np.rad2deg(resta))
		if resta < 10e-8:
			#print('partida',point)
			pass
		else:
			points_list.append(point)
			resta_list.append(resta)
	#print('final lista',points_list)
	vPartida = np.array([P[0]-centro[0],P[1]-centro[1]]) 
	#print('vector partida',vPartida)
	anguloPoint_list = []
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
		anguloPoint_list.append(anguloPoint)
		#print('angulo',np.rad2deg(anguloPoint))	
	#print(np.rad2deg(anguloPoint_list))
	choque = points_list[np.argmin(anguloPoint_list)]
	#print('STEM choque:',choque)
	hit = choque
	P = choque
	#tx = np.append(tx,choque[0])
	#ty = np.append(ty,choque[1])###
	#	hallando la velocidad
	###

	if P[1] == 0.0:
		#print('choque techo')
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)
		vReflejado = np.array([vIncidente[0], -vIncidente[1]])
	elif P[0] == -base/2:
		#print('choque pared izquierda')
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)
		vReflejado = np.array([-vIncidente[0], vIncidente[1]])
	elif P[1] == -altura:
		#print('choque base')
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)
		vReflejado = np.array([vIncidente[0], -vIncidente[1]])
	elif P[0] == base/2:
		#print('choque pared derecha')
		d = hit - centro
		vIncidente = rotar(d,np.pi/2)
		vReflejado = np.array([-vIncidente[0], vIncidente[1]])
	else:
		print('problemas!!')
		
		
	#plt.arrow(P[0],P[1],vIncidente[0],vIncidente[1])
	#plt.arrow(P[0],P[1],vReflejado[0],vReflejado[1])	 
		
	v = vReflejado
		
	##
	A2 = P
	#para A
	#thetaA1 = thetaAngle(np.array([A1[0]-centro[0],A1[1]-centro[1]]))
	#print('angulo 1 punto A',np.rad2deg(thetaA1))
	#para B
	#thetaA2 = thetaAngle(np.array([A2[0]-centro[0],A2[1]-centro[1]]))
	#print('angulo 2 punto B',np.rad2deg(thetaA2))
			
	#if thetaA1 < thetaA2:
		#theta = np.linspace(0,thetaB-thetaA,num=10)	
	#	thetaList = np.linspace(thetaA1,thetaA2,num=20)
	#else: #thetaA > thetaB
	#	thetaList = np.linspace(thetaA1,thetaA2+2*np.pi,num=20)  
	#theta = np.linspace(0,thetaB-thetaA,num=10)
	#print(np.rad2deg(thetaList))
	#R = np.matrix([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
	#	curveX = R*np.cos(thetaList)+centro[0]
	#	curveY = R*np.sin(thetaList)+centro[1]
	#	plt.plot(curveX,curveY,color = 'b', linewidth=1)		
	##

	if P[1] == 0.0:
		#print('exit!')
		#plt.plot(P[0],P[1],'kX')
		v = vIncidente
		#print(P,v)
			
		#break
	#print(tx,ty)        #print(i,P[0],P[1])
	return [P,v]#return [[tx,ty],v]


#print(stemOneMove(1,1,0.1,np.array([-0.5,-0.1]),np.array([0.1,0.1])))
