import numpy as np
from trajetoria import trajetoria
import rospy
from std_msgs.msg import Float64
from trajetoriaPes import trajetoriaPes
from trajetoriaPesInicio import trajetoriaPesInicio
from globalVariables import GlobalVariables
from CalculoMhd import calculoMhd
from Controles import controles
from fase1 import fase1

angle = Float64() #variable that will be published

#configuring ROS node###############################################################

rospy.init_node('angles') #iniciando o nó
#ang 1-6 perna direita e ang7-12 perna esquerda

p1 = rospy.Publisher('/robotis_op3/r_hip_yaw_position/command',Float64,queue_size=1) # '/robotis_op3/r_hip_yaw_position/command' é o tópico onde a mensagem será publicada"
p2 = rospy.Publisher('/robotis_op3/r_hip_roll_position/command',Float64,queue_size=1)
p3 = rospy.Publisher('/robotis_op3/r_hip_pitch_position/command',Float64,queue_size=1)
p4 = rospy.Publisher('/robotis_op3/r_knee_position/command',Float64,queue_size=1)
p5 = rospy.Publisher('/robotis_op3/r_ank_roll_position/command',Float64,queue_size=1)
p6 = rospy.Publisher('/robotis_op3/r_ank_pitch_position/command',Float64,queue_size=1)
p7 = rospy.Publisher('/robotis_op3/l_hip_yaw_position/command',Float64,queue_size=1)
p8 = rospy.Publisher('/robotis_op3/l_hip_roll_position/command',Float64,queue_size=1)
p9 = rospy.Publisher('/robotis_op3/l_hip_pitch_position/command',Float64,queue_size=1)
p10 = rospy.Publisher('/robotis_op3/l_knee_position/command',Float64,queue_size=1)
p11 = rospy.Publisher('/robotis_op3/l_ank_roll_position/command',Float64,queue_size=1)
p12= rospy.Publisher('/robotis_op3/r_ank_pitch_position/command',Float64,queue_size=1)
publishers = np.array([p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12])
#-----------------------------------------------------------
#Obter todas as trajeotrias do CoM
#-----------------------------------------------------------
#Darwin
xod  = 0.00
yod  = 0.035
zod  = 0.22
dxod = 0.24
dyod = 0.00
dzod = 0.00
X0 = np.array([[xod],[yod],[zod],[dxod],[dyod],[dzod]]) #condição inicial
#-----------------------------------------------------------
#variável de controle inicial
#modificar os valores obtidos aqui
#-----------------------------------------------------------
expK   = 1000 #Darwin
#DArwin dynamic model parameters
phi= 0.6482570031
theta = 0.2823507428
k = 153.9300628927
Bss = 0.0414743461

U0 = np.zeros((5,1))
k = k/expK #tratar o valor 
U0[0,0] = theta
U0[1,0] = phi
U0[2,0] = k 
U0[3,0] = Bss
U0[4,0] = expK

[PA,PB,PC,trajCoM1,indContadoPe] = trajetoria(U0,X0) #trajetória para o CoM

quatidadePassos = 3

#Ganhos LQR (perna direita) e proporcional (perna esquerda)
ganhoS1 = 0
ganhoQ1 = 20
ganhoR1 = 0.00001
ganhoK1 = 9000

#Ganhos LQR (perna esquerda) e proporcional (perna direita)
ganhoS2 = 0
ganhoQ2 = 20
ganhoR2 = 0.00001
ganhoK2 = 9000

vecGanho1 = np.array([ganhoS1, ganhoQ1, ganhoR1, ganhoK1]).reshape((4,1))
vecGanho2 = np.array([ganhoS2, ganhoQ2, ganhoR2, ganhoK2]).reshape((4,1))

#trajetoria 1
#[trajPB1, trajPB, trajPA, trajCoM1, trajCoM2, trajCoM3, passoTrajCoM, passoComprimento] = trajetoria(U0,X0)
#trajetoria 2
CoM = trajCoM1
print(np.size(trajCoM1))
ind = np.size(CoM,0) #pegar a última posição do vetor de pontos
trajCoM2 = np.zeros((ind,3))
trajCoM3 = np.zeros((ind,3))
offsetx = CoM[ind-1,0]#cálcular o offset em x
offsety = CoM[ind-1,1]#calcular o offset em y
trajCoM2[:,0] = -CoM[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
trajCoM2[:,1] = -CoM[range(ind-1,-1,-1),1] + 2*offsety #calcular a trajetória simétrica para y
trajCoM2[:,2] =  CoM[range(ind-1,-1,-1),2] #em z não muda


offsetx = trajCoM2[ind-1,0] #cálcular o offset em x
offsety = trajCoM2[ind-1,1] #calcular o offset em y
trajCoM3[:,0] = -trajCoM2[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
trajCoM3[:,1] = trajCoM2[range(ind-1,-1,-1),1] #calcular a trajetória simétrica para y
trajCoM3[:,2] =  trajCoM2[range(ind-1,-1,-1),2] #em z não muda

trajCoM2 = np.concatenate((trajCoM2, trajCoM3),axis=0)

passoTrajCoM = trajCoM2[np.size(trajCoM2,0)-1,0] - trajCoM2[0,0]


#trajetoria3
ind = np.size(trajCoM2,0) #pegar a última posição do vetor de pontos
trajCoM3 = np.zeros((ind,3))
offsetx = trajCoM2[ind-1,0] #cálcular o offset em x
offsety = trajCoM2[ind-1,1] #calcular o offset em y
#clr trajCoM3
trajCoM3[:,0] = -trajCoM2[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
trajCoM3[:,1] = -trajCoM2[range(ind-1,-1,-1),1] + 2*offsety #calcular a trajetória simétrica para y
trajCoM3[:,2] =  trajCoM2[range(ind-1,-1,-1),2] #em z não muda
    
#-----------------------------------------------------------
#Trajetória dos pés
#-----------------------------------------------------------
passoComprimento = PB[0,0] #tamanho do passo
passoLargura     = PB[1,0] #Largura do passo
passoAltura      = 0.0    #altura de cada passo

#trajetoria pé B inicial
tamTrajPeB1 = indContadoPe
#trajPB1 = trajetoriaPes(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,1,tamTrajPeB1)
trajPB1 = trajetoriaPesInicio(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,tamTrajPeB1)


passoComprimento2 = PB[0,0] #tamanho do passo
passoLargura2     = 0 #Largura do passo
passoAltura2      = 0.0   #altura de cada passo

#trajetoria pé A
tamTrajPa= (np.size(trajCoM1,0)+indContadoPe)/2
trajPA = trajetoriaPes(np.array([[passoComprimento2+passoComprimento2],[passoLargura2],[0]]),passoComprimento2,passoAltura2,0,tamTrajPa)
#trajPA = np.asarray(trajPA)
#trajtoria  pé B
trajPB = trajPA
#k = np.size(trajPB,0)
#for i in range(k):
trajPB[:,0] = trajPB[:,0] + passoComprimento
trajPB[:,1] = passoLargura

#--------------------------------------------------------------------------------------
#Modelo do robô 
#primeiros testes 
#implementação começa aqui
#--------------------------------------------------------------------------------------
#parâmetros necessarios
glob = GlobalVariables()
#Parametros de D.H. Perna- tabela do artigo
thetaR = glob.getOr()
thetaL = glob.getOl()
theta = np.concatenate((thetaR,thetaL),axis=1) # parametros variaveis
tempo = 0

        
#primeira parte da caminhdada
phase = 1
passos = 1
trajCoM = trajCoM1
trajP = trajPB1
T = np.size(trajCoM,0) #o tamanho de trajCoM = ind
[ha, ha2, hP,Mhd, Mhd2, Mdhd, Mdhd2, tempo1] = calculoMhd(trajCoM,theta,trajP,phase) 
[ha, ha2, theta, Mtheta, Mtheta2] = controles(theta,ha,ha2,hP,Mhd2, Mdhd2,Mhd,Mdhd,vecGanho2,T,phase,publishers)

tempo = tempo+tempo1
i = 0   
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    if passos >=quatidadePassos:
        break

    #fase 2
    trajCoM = trajCoM2
    T = np.size(trajCoM,0) #o tamanho de trajCoM = ind
    #for j in range(np.size(trajCoM,0)):
    trajCoM[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
    trajP = trajPA
    #k = np.size(trajP,0)
    #for j in range(k):
    trajP[:,0] = trajP[:,0]+ i*2*passoComprimento
    phase = 2
    passos = passos + 1
    [haInutil, ha2Inutil,hP, Mhd, Mhd2, Mdhd, Mdhd2, tempo2] = calculoMhd(trajCoM,theta,trajP,phase) 
    [ha, ha2, theta, Mtheta, Mtheta2] = controles(theta,ha,ha2,hP,Mhd2, Mdhd2,Mhd,Mdhd,vecGanho2,T,phase,publishers)
    # [ha,ha2,theta,tempo2,Mtheta, Mtheta2] = fase2(ha,ha2,trajCoM,np.size(trajPA,0),trajP,theta,tempo,vecGanho2)
    if passos >=quatidadePassos: #é a quantidade de passos da trajetória desejada
        break
    
    #fase 3
    tempo=tempo+tempo2
    trajCoM = trajCoM3
    T = np.size(trajCoM,0) #o tamanho de trajCoM = ind
    #for j in range(np.size(trajCoM,0)):
    trajCoM[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
    trajP = trajPB
    #k = np.size(trajP,0)
    #for j in range(k):
    trajP[:,0] = trajP[:,0]+ i*2*passoComprimento
    #A cada 2 passos ou quando a velocidade muda, é necessário chamar o 
    #LQR do modelo 3D
    #un = LRQ3D(U0,X0,passos,tempo,xn,un,PC) 
    #quem serão xn e pc??????????????????????????????????????????????????
    #[PA,PB,PC,trajCoM,indContadoPe] = trajetoria(un,xn)
    phase = 3   
    passos = passos + 1
    [haInutil, ha2Inutil, hP, Mhd, Mhd2, Mdhd, Mdhd2, tempo3] = calculoMhd(trajCoM,theta,trajP,phase) 
    [ha, ha2, theta, Mtheta, Mtheta2] = controles(theta,ha,ha2,hP,Mhd2, Mdhd2,Mhd,Mdhd,vecGanho2,T,phase,publishers)
    if passos >=quatidadePassos:
        break
    
    tempo=tempo+tempo3
    i = i+1


