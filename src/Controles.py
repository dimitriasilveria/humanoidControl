import numpy as np
from kinematicRobo import kinematicRobo
from jacobianoPes import jacobianoPes
from dualHamiltonOp import dualHamiltonOp
from dualQuatConj import dualQuatConj
from dualQuatMult import dualQuatMult
from plotGraficosControle import plotGraficosControle
from getRotationDualQuat import getRotationDualQuat
from getPositionDualQuat import getPositionDualQuat
from jacobianoCoM import jacobiano2
from globalVariables import GlobalVariables
from kinematicModel import KinematicModel
import rospy
from std_msgs.msg import Float64

import math as mt

def controles(theta,ha,ha2,hP,Mhd2, Mdhd2,Mhd,Mdhd,vecGanho,T,phase,publishers):

    if phase == 2: 
        hP = ha2
        hOrg = np.array([[1],[0], [0], [0], [0], [0], [0], [0]]) #posição da base
        ha2 = kinematicRobo(theta,hOrg,hP,0,0) #posição da perna direita
    else:
        if phase ==3:
            hP = ha2
    glob = GlobalVariables()
    dt  = glob.getHEDO()
    MDH = glob.getMDH()

    angleMsg = Float64()

    mhd = np.zeros((8,1))
    mdhd = np.zeros((8,1))
    mhd2 = np.zeros((8,1))
    mdhd2 = np.zeros((8,1))
    
    angle = np.zeros(T)
    angled = np.zeros(T)
    angle2 = np.zeros(T)
    angled2 = np.zeros(T)

    Pos = np.zeros((3,T))
    Posd = np.zeros((3,T))
    Pos2 = np.zeros((3,T))
    Posd2 = np.zeros((3,T))

    Mha2 = np.zeros((8,T))
    Mha = np.zeros((8,T))
    Mtheta2 = np.zeros((6,T))
    Mtheta = np.zeros((6,T))
    
    #LQR 
    #calculo dos parâmetros
    ganhoS = vecGanho[0,0]
    ganhoQ = vecGanho[1,0]
    ganhoR = vecGanho[2,0]
    #controlador proporcional
    ganhoK2 = vecGanho[3,0]
    K2 = ganhoK2*np.eye(8)

    S = ganhoS*np.eye(8)
    Q = ganhoQ*np.eye(8)
    R = ganhoR*np.eye(8)
    Rinv = np.linalg.inv(R)
    # print('R::',Rinv)
    # return
    ab = np.array([1, -1,-1, -1, 1, -1, -1,-1])
    C8 = np.diag(ab)
    hOrg = np.array([[1],[0], [0], [0], [0], [0], [0], [0]]) #posição da base

    #iniciar condições finais esperadas para P e E
    Pf = S
    Ef = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((8,1))
    P = Pf
    MP2 = np.zeros((8,8,T))
    for j in range(8):
        for k in range(8):
            MP2[j,k,T-1] = P[j,k]
    E = Ef
    ME2 = np.zeros((8,T))
    for j in range(8):
        ME2[j,T-1] = E[j,0]

    #calcular matrizes de ganho
    for i in range(T-2,-1,-1):   

        for j in range(8):
            mhd[j,0] = Mhd[j,i+1] 
            mdhd[j,0] = Mdhd[j,i+1]
        aux = dualQuatMult(dualQuatConj(mhd),mdhd)
        A  = dualHamiltonOp(aux,0)
        c = -aux
        # prod2 = np.dot(P,Rinv)
        P = P -(-P@A -A.T@P + P@Rinv@P - Q)*dt
        for j in range(8):
            for k in range(8):
                MP2[j,k,i] = P[j,k]
        E = E - ((-1)*(A.T)@E + P@Rinv@E - P@c)*dt
        for j in range(8):
            ME2[j,i] = E[j,0]
  
    for i in range(0,T,1):
        #tic = tm.time()
        #Controlador LQR para o CoM
        #calculo de A e c
        for j in range(8):
            mhd[j,0] = Mhd[j,i] 
            mdhd[j,0] = Mdhd[j,i]
        aux = dualQuatMult(dualQuatConj(mhd),mdhd) #calculo de hd conjugado * hd derivada
        A  = dualHamiltonOp(aux,0)
        c = -aux
        #inicio do controlador
        #hB_O6a = dualQuatMult(hOrg,hP)
        xe = KinematicModel(MDH,theta,6,0)
        if(phase == 1 | phase == 3):
            Ja = jacobiano2(theta,hOrg,hP,xe,0) #jacobiano para a perna direita
        else:   
            Ja = jacobiano2(theta,hOrg,hP,xe,1) 
        # Ja = jacobianoCinematica(theta,hOrg,hP,1,1)
        #calculo de P e E
        #calculo de N   
        Hd  = dualHamiltonOp(mhd,0)
        # prod3 = np.dot(Hd,C8)
        N  = Hd@C8@Ja
        #pseudo inversa de N
        Np  = np.linalg.pinv(N)

        #calculo do erro
        e  = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha),mhd)
        #calculo de P e E

        E[:,0] = ME2[:,i]
        P[:,:] = MP2[:,:,i].reshape((8,8))
        #Pxe= np.dot(P,e)
        #do = Np@Rinv@(P@e + E) 
        do = Np@Rinv@(P@e + E) #equação final para theta ponto
        #calculo do theta deseja
        od  = (do*dt)/2
        #print('od', od)
        if(phase == 1 or phase == 3):
            theta[:,0] = theta[:,0] + od[:,0]
        else:
            theta[:,1] = theta[:,1] + od[:,0]

		#o movimento dos motores é limitado entre pi/2 e -pi/2, então, se theta estiver
		#fora do intervalo, esse for faz theta = limite do intervalo
        # for j in range(0,6,1):
        #     if abs(theta[j,0]) > hpi:
        #         theta[j,0] = np.sign(theta[j,0])*hpi
        if(phase == 1 or phase == 3):
            ha  = kinematicRobo(theta,hOrg,hP,1,1)  #posição do CoM com perna direita apoiada
        else:
            ha  = kinematicRobo(theta,hOrg,hP,0,1) #posição do CoM com perna esquerda apoiada

        #plotar os dados
        for j in range(8):
            Mha[j,i] = ha[j,0]
        #posição
        PosAux = getPositionDualQuat(ha)
        PosdAux = getPositionDualQuat(mhd)
        for j in range(3):
            Pos[j,i]  = PosAux[j,0] #retorna a posição x,y e z de ha
            Posd[j,i] = PosdAux[j,0] #retorna a posição x,y e z de todos os hd
        #orientação
        ra = getRotationDualQuat(ha) ##extrai o vetor do dual quat que representa a rotação
        rd = getRotationDualQuat(mhd)
        co = mt.acos(ra[0,0])
        angle[i] = co
        co = mt.acos(rd[0,0])
        angled[i] = co
        for j in range(6):
            Mtheta[j,i] = theta[j,0]

        #controlador 2 para os pés (proporcional com feed forward)
        #calculo de A e c
        #é necessário??????????????????????????????????????????????????????????????????
        #aux2 = dualQuatMult(dualQuatConj(Mhd2(:,i)),Mdhd2(:,i));
        #A2  = dualHamiltonOp(aux2,0);
        #c = -aux2;
        #inicio do controlador  
        #hB_O6a = dualQuatMult(hOrg,hP)
        xe2 = kinematicRobo(theta,hOrg,hP,1,0)
        Ja2 = jacobianoPes(theta,ha,xe2,1)
        #Ja2 = jacobianoCinematica(theta,hOrg,hP,1,0)
        #calculo de P e E
        #calculo de N  
        mhd2 = np.zeros((8,1))
        mdhd2 = np.zeros((8,1))
        for j in range(8):
            mhd2[j,0] = Mhd2[j,i] 
            mdhd2[j,0] = Mdhd2[j,i]
        Hd2  = dualHamiltonOp(mhd2,0)
        N2  = Hd2@C8@Ja2
        #pseudo inversa de N
        Np2  = np.linalg.pinv(N2)
        
        #calculo do erro
        e2  = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha2),mhd2)
        
        vec2 = dualQuatMult(dualQuatConj(ha2),mdhd2)
        #
        # K2xe2 = np.dot(K2,e2)
        do2 = Np2@(K2@e2-vec2)
        od2  = (do2*dt)/2
        if(phase == 1 or phase == 3):
            theta[:,1] = theta[:,1] + od2[:,0]
        else:
            theta[:,0] = theta[:,0] + od2[:,0]
        # for j in range (0,6,1):
        #     if abs(theta[j,1]) > hpi:
        #         theta[j,1] = np.sign(theta[j,1])*hpi
        print(theta)
        # publicando os ângulos nas juntas
        angleMsg.data = theta[0,0]#escrevendo theta[0] em angle
        publishers[0].publish(angleMsg) #publicando angle

        angleMsg.data = theta[1,0]
        publishers[1].publish(angleMsg)

        angleMsg.data = theta[2,0]
        publishers[2].publish(angleMsg)

        angleMsg.data = theta[3,0]
        publishers[3].publish(angleMsg)

        angleMsg.data = theta[4,0]
        publishers[4].publish(angleMsg)

        angleMsg.data = theta[5,0]
        publishers[5].publish(angleMsg)

        angleMsg.data = theta[0,1]
        publishers[6].publish(angleMsg)

        angleMsg.data = theta[1,1]
        publishers[7].publish(angleMsg)

        angleMsg.data = theta[2,1]
        publishers[8].publish(angleMsg)

        angleMsg.data = theta[3,1]
        publishers[9].publish(angleMsg)

        angleMsg.data = theta[4,1]
        publishers[10].publish(angleMsg)

        angleMsg.data = theta[5,1]
        publishers[11].publish(angleMsg)

        rospy.sleep(1)
		
        if(phase == 1 or phase == 3):
            ha2  = kinematicRobo(theta,hOrg,hP,1,0)
        else:
            ha2  = kinematicRobo(theta,hOrg,hP,0,0) #posição da perna direita
        
        #plotar os dados
        Mha2[:,i]  = ha2[:,0]
        #posição
        Pos2Aux = getPositionDualQuat(ha2)
        Posd2Aux = getPositionDualQuat(mhd2)
        #for j in range(3):
        Pos2[:,i]  = Pos2Aux[:,0] 
        Posd2[:,i] = Posd2Aux[:,0] 

        #orientação
        ra = getRotationDualQuat(ha2)
        rd = getRotationDualQuat(mhd2)
        co = mt.acos(ra[0,0])
        angle2[i] = co
        co = mt.acos(rd[0,0])
        angled2[i] = co
        for j in range(6):
            Mtheta2[j,i] = theta[j,1]

    

        #mostrar no console o andamento do metódo
        #toc = tm.time() - t #elapsed
        #msg = print('#d de  #d | tempo (s): #f',i,T,toc);
        #disp(msg);
    t1 = 0
    #plotGraficosControle(t1,dt,T,Pos,Posd,angle,angled,Mha,Mhd,Mtheta,Pos2,Posd2,angle2,angled2,Mha2,Mhd2,Mtheta2,'b','r')
    return ha, ha2, theta, Mtheta, Mtheta2