#! /usr/bin/env python

import numpy as np
from caminhada import caminhada
from globalVariables import GlobalVariables
from caminhada2 import caminhada2
import time
import rospy
from std_msgs.msg import Float64
from globalVariables import GlobalVariables

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

#here ends ROS node configuring###########################################

begin = time.time()
glob = GlobalVariables()
m = glob.getM()
L = glob.getL()
g = glob.getG()
h = glob.getH()
hEdo = glob.getHEDO()
#global  m, L, g,  h, hEdo

U0 = np.zeros((5,1))

#-----------------------------------------------------------
#condição inicial para MS
#-----------------------------------------------------------
# xod  = 0.0001 #x inicial
# yod  = 0.05 #y inicial
# zod  = 0.6 #z inicial 
# dxod = 0.7 #velocidade desejada no MS
# dyod = 0.00 #condição de balanço
# dzod = 0.00 #velocidade em z (igual a zero condição necessaria)

#Darwin
xod  = 0.00
yod  = 0.035
zod  = 0.22
dxod = 0.24
dyod = 0.00
dzod = 0.00


#expK   = 10000 #ordem de grandeza da constante massa-mola Hubo
expK   = 1000 #Darwin

#-----------------------------------------------------------
#variável de controle inicial
#modificar os valores obtidos aqui
#-----------------------------------------------------------

#1m/s
#HUBO
# phi = 0.3408040779
# theta = 0.4052906627
# k = 19196.8680322965
# Bss = 0.0415640571

#DArwin dynamic model parameters
phi= 0.6482570031
theta = 0.2823507428
k = 153.9300628927
Bss = 0.0414743461

#Ganhos LQR (perna direita) e proporcional (perna esquerda)
ganhoS1 = 0
ganhoQ1 = 1
ganhoR1 = 0.000001
ganhoK1 = 1000

#Ganhos LQR (perna esquerda) e proporcional (perna direita)
ganhoS2 = 0
ganhoQ2 = 1
ganhoR2 = 0.000001
ganhoK2 = 1000

#quantidade de passos
quatidadePassos = 3
tam = quatidadePassos
#-----------------------------------------------------------
#Não modificar aqui
#-----------------------------------------------------------

k = k/expK #tratar o valor 
U0[0,0] = theta
U0[1,0] = phi
U0[2,0] = k 
U0[3,0] = Bss
U0[4,0] = expK

X0 = np.array([[xod],[yod],[zod],[dxod],[dyod],[dzod]]) #condição inicial

vecGanho1 = np.array([ganhoS1, ganhoQ1, ganhoR1, ganhoK1]).reshape((4,1))
vecGanho2 = np.array([ganhoS2, ganhoQ2, ganhoR2, ganhoK2]).reshape((4,1))

#caminhada2(U0,X0,quatidadePassos,vecGanho1,vecGanho2)
#Aqui começa a caminhada#########################################################

from trajetoria import trajetoria
from trajetoriaPes import trajetoriaPes
from fase3 import fase3
from fase1 import fase1
from fase2 import fase2
from globalVariables import GlobalVariables
from trajetoriaPesInicio import trajetoriaPesInicio
import numpy as np
from LQR3D import LRQ3D
#-----------------------------------------
#Executar a caminhada
#Parâmetros:
#U:Vetor com as variáveis de controle
#X:Vetor com a condição inicial
#vecGanho1:vetor com os ganhos do controlador para a condição da perna
#          direita em contato com chão
#vecGanho2:vetor com os ganhos do controlador para a condição da perna
#          esquerda em contato com o chão
#------------------------------------------

#-----------------------------------------------------------
#Obter todas as trajeotrias do CoM
#-----------------------------------------------------------
rate = rospy.Rate(1)

while not rospy.is_shutdown():

    [PA,PB,PC,trajCoM1,indContadoPe] = trajetoria(U0,X0) #trajetória para o CoM

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
    passoAltura      = 0.2    #altura de cada passo

    #trajetoria pé B inicial
    tamTrajPeB1 = indContadoPe
    #trajPB1 = trajetoriaPes(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,1,tamTrajPeB1)
    trajPB1 = trajetoriaPesInicio(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,tamTrajPeB1)


    passoComprimento2 = PB[0,0] #tamanho do passo
    passoLargura2     = 0 #Largura do passo
    passoAltura2      = 0.2    #altura de cada passo

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
    #implementação começa aquiiii
    #--------------------------------------------------------------------------------------
    #parâmetros necessarios
    glob = GlobalVariables()
    #hpi = glob.getHpi()
    #L1 = glob.getL1()
    #L2 = glob.getL2()
    #L3 = glob.getL3()
    #L4 = glob.getL4()
    #L5 = glob.getL5()
    #height = glob.getHeight()
    #MDH = glob.getMDH()
    #global hpi, L1, L2, L3, L4, L5, height, MDH 
    #hpi = np.pi/2
    #N = 6 #Numero de Juntas
    #Comprimento das Juntas
    #parametros hubo
    #L1 = 0.085
    #L2 = 0.0 #L2 = 0.182;#modificando L2 para que L seja 1 depois rodar a simulação de novo
    #L3 = 0.3
    #L4 = 0.3
    #L5 = 0.0663
    #height = L2+L3+L4+L5 #altura inicial total do robô pé para o centro de massa
    #Parametros de D.H. Perna- tabela do artigo
    thetaR = glob.getOr()
    thetaL = glob.getOl()
    theta = np.concatenate((thetaR,thetaL),axis=1) # parametros variaveis
    tempo = 0

          
    #primeira parte da caminhdada
    #print('primeiro passinho')
    passos = 1
    [ha,ha2,theta,tempo1, Mtheta, Mtheta2] = fase1(trajCoM1,indContadoPe,trajPB1,theta,vecGanho1)
    for j in range(np.size(Mtheta,1)):
        Vr = Mtheta[:,j].reshape((6,1))   #vetor perna direira p1-6
        Vl = Mtheta2[:,j].reshape((6,1))   #vetor perna esquerda p7-12
        print("shape",np.shape(Vr))

        angle.data = Vr[0,0] #escrevendo Vr[0] em angle
        p1.publish(angle) #publicando angle
        print("publicando", Vr)
        angle.data = 0.05
        p2.publish(angle)

        angle.data = Vr[2,0]
        p3.publish(angle)

        angle.data = Vr[3,0]
        p4.publish(angle)

        angle.data = Vr[4,0]
        p5.publish(angle)

        angle.data = Vr[5,0]
        p6.publish(angle)

        angle.data = Vl[0,0]
        p7.publish(angle)

        angle.data = Vl[1,0]
        p8.publish(angle)

        angle.data = Vl[2,0]
        p9.publish(angle)

        angle.data = Vr[3,0]
        p10.publish(angle)

        angle.data = Vl[4,0]
        p11.publish(angle)

        angle.data = Vl[5,0]
        p12.publish(angle)

        rospy.sleep(1)

    # if passos >=tam:
    #     return

    tempo = tempo+tempo1
    i = 0   
    while 1:
        
        trajCoM = trajCoM2
        #for j in range(np.size(trajCoM,0)):
        trajCoM[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
        trajP = trajPA
        #k = np.size(trajP,0)
        #for j in range(k):
        trajP[:,0] = trajP[:,0]+ i*2*passoComprimento

        passos = passos + 1
        [ha,ha2,theta,tempo2, Mtheta, Mtheta2] = fase2(ha,ha2,trajCoM,np.size(trajPA,0),trajP,theta,tempo,vecGanho2)
        # if passos >=tam: #tam é a quantidade de passos da trajetória desejada
        #     return
        
        tempo=tempo+tempo2
        trajCoM = trajCoM3
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
        passos = passos + 1
        [ha,ha2,theta,tempo3, Mtheta, Mtheta2] = fase3(ha,ha2,trajCoM,np.size(trajPB,0),trajP,theta,tempo,vecGanho1)
        
        # if passos >=tam:
        #     return
        
        tempo=tempo+tempo3
        i = i+1
        
        
    #Aqui termina a caminhada#####################################3

    end = time.time()
    print('execution time:', end - begin)