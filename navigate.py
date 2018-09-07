import vrep, time, cv2
import numpy as np
server_IP = "127.0.0.1"
server_port = 25000
nome_sensor = []
handle_sensores = []
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6]
braitenbergR=[-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2]
detect = [0,0,0,0,0,0,0,0]
noDetectionDist=0.5
maxDetectionDist=0.2
#---------------------Conecta no servidor---------------------------------
clientID = vrep.simxStart(server_IP, server_port, True, True, 2000, 5)

if (clientID!=-1):
	print ("Servidor Conectado!")
	

#------------------------------Inicializa Sensores ----------------------------
	for i in range(0,8):
		nome_sensor.append("Pioneer_p3dx_ultrasonicSensor" + str(i+1))

		res, handle = vrep.simxGetObjectHandle(clientID, nome_sensor[i], vrep.simx_opmode_oneshot_wait)

		if(res != vrep.simx_return_ok):
			print (nome_sensor[i] + " nao conectado")
		else:
			print (nome_sensor[i] + " conectado")
			handle_sensores.append(handle)
			
	#Vision sensor		
	res, visionHandle = vrep.simxGetObjectHandle(clientID, "Vision_sensor", vrep.simx_opmode_oneshot_wait)		
			
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


else:
	print("Servidor desconectado!")

			
def ler_distancias(sensorHandle):

	"""
		Esse metodo ira ler a distancia de um conjunto de sensores ultrassonicos
		parametro: handle dos sensores
		retorna:   distancias em metros
	"""
	distancias = []
	
	for sensor in sensorHandle:
		returnCode, detectionState, detectedPoint,_,_ = vrep.simxReadProximitySensor(clientID, sensor, vrep.simx_opmode_streaming)
		if (returnCode == vrep.simx_return_ok):
			if(detectionState != 0):
				distancias.append(round(detectedPoint[2],5))
			else:
				#Ponto muito distante, considerei 5 metros para nao pegar ruido
				distancias.append(0.5)
		else:
			#print ("Erro no sensor "+str(i+1))
			time.sleep(0.1)
	return distancias
	
display = False
#------------------------------ Loop principal ----------------------------
while vrep.simxGetConnectionId(clientID) != -1:
	vLeft = vRight = 5
	dist = ler_distancias(handle_sensores)
	"""
	#codigo pra ler da camera
	code, resolution, image = vrep.simxGetVisionSensorImage(clientID, visionHandle, 0, vrep.simx_opmode_streaming)
	if code == vrep.simx_return_ok:
		if not display:
		    img = np.array(image,dtype=np.uint8)
		    img.resize([resolution[1],resolution[0], 3])
		    cv2.imshow('image',img)
		    display=True
		    if cv2.waitKey(1) & 0xFF == ord('q'):
		        break
	"""
	

	
	"""
	
	if dist:
		for i in range(len(dist)):
			if(dist[i] < noDetectionDist):
			    detect[i] = 1 - ((dist[i]-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
			else:
			    detect[i]=0
		
		for i in range(8):
			vLeft = vLeft + braitenbergL[i]*detect[i]
			vRight = vRight+ braitenbergR[i]*detect[i]
	
	
	

		vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vLeft, vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vRight, vrep.simx_opmode_streaming)
	"""
	
	
