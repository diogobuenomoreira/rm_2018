# -*- coding: utf-8 -*-
import vrep, time, math
import numpy as np


server_IP = "127.0.0.1"
server_port = 25000
nome_sensor = []
handle_sensores = []

detect = np.zeros(6)
noDetectionDist = 4.0
#maxDetectionDist=0.1


#---------------------Conecta no servidor---------------------------------


clientID = vrep.simxStart(server_IP, server_port, True, True, 2000, 5)

if (clientID!=-1):
	print ("Servidor Conectado!")
	

#------------------------------Inicializa Sensores ----------------------------
	for i in range(0,6):
		nome_sensor.append("Pioneer_p3dx_ultrasonicSensor" + str(i+1))

		res, handle = vrep.simxGetObjectHandle(clientID, nome_sensor[i], vrep.simx_opmode_oneshot_wait)

		if(res != vrep.simx_return_ok):
			print (nome_sensor[i] + " nao conectado")
		else:
			print (nome_sensor[i] + " conectado")
			handle_sensores.append(handle)
			
		
#------------------------------Inicializa Motores ----------------------------
	resLeft, handle_motor_esq = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
	if(resLeft != vrep.simx_return_ok):
		print("Motor Esquerdo : Handle nao encontrado!")
	else:
		print("Motor Esquerdo: Conectado")

	resRight, handle_motor_dir = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait)
	if(resRight != vrep.simx_return_ok):
		print("Motor Direito: Handle nao encontrado!")
	else:
		print("Motor Direito: Conectado")

#------------------------------Inicializa Robo ----------------------------

	resRobo, handle_robo = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx", vrep.simx_opmode_oneshot_wait)


else:
	print("Servidor desconectado!")


def ler_distancias(sensorHandle):
    distancias=[]
    for sensor in sensorHandle:
        returnCode, detectionState, detectedPoint,_,_ = vrep.simxReadProximitySensor(clientID, sensor, vrep.simx_opmode_streaming)
        if (returnCode == vrep.simx_return_ok):
            if(detectionState != 0):
                distancias.append(round(detectedPoint[2],5))
            else:
				#Muito distante
                distancias.append(noDetectionDist)
        else:
			#print ("Erro no sensor "+str(i+1))
            time.sleep(0.1)
    return distancias


reference = 0.5
Kp = 1
right_vel = 1
left_vel = 0.8
flag = False

#------------------------------ Loop principal ----------------------------
while vrep.simxGetConnectionId(clientID) != -1:
    while(1):
        dist = ler_distancias(handle_sensores)
        if(dist!=[]):
            if(min(dist)<=reference):
                '''measured = max(dist[0],dist[1],dist[2])
                error = abs(reference - measured)
                u = Kp * error
                vel = u
                if(vel<0.5):
                    vel=0.5'''
                vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, -1 , vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, 1 , vrep.simx_opmode_streaming)
                flag = True
                        
            elif(flag == False):
                measured = min(dist[3],dist[4],dist[5])
                error = abs(reference - measured)
                u = Kp * error
                vel = u
                if(vel<0.5):
                    vel=0.5
                #vel_right = right_vel + vel
                #vel_left = left_vel + vel
                vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vel , vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vel, vrep.simx_opmode_streaming)
            else:
                measured = max(dist[0],dist[1],dist[2])
                error = abs(reference - measured)
                u = Kp * error
                vel = u
                if(vel<0.5):
                    vel=0.5
                #vel_right = right_vel + vel
                #vel_left = left_vel + vel
                vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vel , vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vel, vrep.simx_opmode_streaming)
            
            
        
               
			
	

