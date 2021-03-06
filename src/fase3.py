import numpy as np 
import math as mt 
from jacobianoCinematica import jacobianoCinematica
from kinematicRobo import kinematicRobo
from transformacao import transformacao
from globalVariables import GlobalVariables
from getPositionDualQuat import getPositionDualQuat
from getRotationDualQuat import getRotationDualQuat
from dualQuatConj import dualQuatConj
from dualHamiltonOp import dualHamiltonOp
from dualQuatMult import dualQuatMult
from plotGraficosControle import plotGraficosControle
from jacobianoCoM import jacobiano2
from jacobianoPes import jacobianoPes
from kinematicModel import KinematicModel
#---------------------------------------------
#Método para executar o  passo com a perna direita como suporte da 
#caminhada e a perna esquerda em movimento
#---------------------------------------------
def fase3(ha,ha2,trajCoM,ind,trajPB,theta,t1,vecGanho):
    # #global hpi, L1, L2, L3, L4, L5, height, MDH, hEdo
    # glob = GlobalVariables()
    # hEdo = glob.getHEDO()
    # L1 = glob.getL1()
    # #L2 = glob.getL2()
    # #L3 = glob.getL3()
    # #L4 = glob.getL4()
    # #L5 = glob.getL5()
    # height = glob.getHeight()
    # MDH = glob.getMDH()
    # hpi = glob.getHpi()
    # dt = hEdo #dt é o tempo da solução da equação Edo
    # hOrg = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) #posição da base 
    # T = np.size(trajCoM,0)
    # #T = 500;
    # #t = 1:1:T;
    # tempo = (T-1)*dt
    # #tempo = (T)*dt
    # #matrizes auxiliares
    # Mhd = np.zeros((8,T))
    # Mha = np.zeros((8,T))
    # Mdhd = np.zeros((8,T))
    # Mtheta = np.zeros((6,T))
    # mhd = np.zeros((8,1))
    # mdhd = np.zeros((8,1))
    # mhd2 = np.zeros((8,1))
    # mdhd2 = np.zeros((8,1))
    # mhdPlus = np.zeros((8,1))
    # mdhdPlus = np.zeros((8,1))

    # Mhd2 = np.zeros((8,T))
    # Mha2 = np.zeros((8,T))
    # Mdhd2= np.zeros((8,T))
    # Mtheta2 = np.zeros((6,T))

    # angle = np.zeros(T)
    # angled = np.zeros(T)
    # angle2 = np.zeros(T)
    # angled2 = np.zeros(T)


    # Pos = np.zeros((3,T))
    # Posd = np.zeros((3,T))
    # Pos2 = np.zeros((3,T))
    # Posd2 = np.zeros((3,T))

    # #calculo de Mhd - matriz de hd
    # r = np.array([1, 0, 0, 0]).reshape(4,1)
    # #p = [0 0 0 0]';
    # p = np.array([0, 0, -L1, -height]).reshape((4,1))
    # hB1 = transformacao(p,r) #transformação base robô
    # for i in range(0,T,1):
    #     p = np.array([0, trajCoM[i,0], trajCoM[i,1], trajCoM[i,2]]).reshape((4,1))
    #     r = np.array([1, 0, 0, 0]).reshape((4,1))
    #     hd = transformacao(p,r)
    #     hd = dualQuatMult(hB1,hd)
    #     mhd = hd
    #     #for j in range(8):
    #     Mhd[:,i] = mhd[:,0]
       
    #     if i <ind:
    #         p = np.array([0, trajPB[i,0], trajPB[i,1], trajPB[i,2]]).reshape((4,1))
    #         n = np.array([0, 1, 0])
    #         angulo = np.pi/2
    #         realR = np.array([mt.cos(angulo/2)])
    #         imagR = mt.sin(angulo/2)*n
    #         rb = np.concatenate((realR,imagR), axis = 0).reshape((4,1))
    #         hd = transformacao(p,rb)
    #         hd = dualQuatMult(hB1,hd)
    #         mhd2 = hd
    #         #for j in range(8):
    #         Mhd2[:,i] = mhd2[:,0]
    #     else:
    #         #for j in range(8):
    #         Mhd2[:,i] = Mhd2[:,ind-1]



    # hP = ha2
   
    # #Mdhd[:,0]  = (Mhd[:,0]  - Mhd[:,0])*(1/dt)
    # #Mdhd2[:,0]  = (Mhd2[:,0]  - Mhd2[:,0])*(1/dt)

    # for i in range(1,T,1):
    #     #for j in range(8):
    #     Mdhd[:,i] =  (Mhd[:,i] - Mhd[:,i-1])*(1/dt)
    #     #Mdhd[:,i] = mdhd[:,0]
    #     Mdhd2[:,0] =  (Mhd2[:,i]- Mhd2[:,i-1])*(1/dt)
    #     #Mdhd2[:,i] = mdhd2[:,0]


    # ##################################
    # #inicio do codigo
    # #LQR
    # ganhoS = vecGanho[0,0]
    # ganhoQ = vecGanho[1,0]
    # ganhoR = vecGanho[2,0]
    # #controlador proporcional
    
    # ganhoK2 = vecGanho[3,0]
    # K2 = ganhoK2*np.eye(8)

    # #ganho P-FF
    # S = ganhoS*np.eye(8)
    # Q = ganhoQ*np.eye(8)
    # R = ganhoR*np.eye(8)
    # Rinv = np.linalg.inv(R)
    # C8 = np.diag([1, -1, -1, -1, 1, -1, -1, -1])
    # #iniciar condições finais esperadas para P e E
    # Pf = S
    # Ef = np.array([0, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1))
    

    # P = Pf
    # MP2 = np.zeros((8,T))
    # MP2 = np.zeros((8,8,T))
    # for j in range(8):
    #     for k in range(8):
    #         MP2[j,k,T-1] = P[j,k]
    # E = Ef
    # ME2 = np.zeros((8,T))
    # #for j in range(8):
    # ME2[:,T-1] = E[:,0]

   
    # for i in range(T-2,0,-1):
    #     #for j in range(8):
    #     mhdPlus[:,0] = Mhd[:,i+1]
    #     mdhdPlus[:,0] = Mdhd[:,i+1]
    #     mhd[:,0] = Mhd[:,i]
    #     mdhd[:,0] = Mdhd[:,i]   
    #     aux = dualQuatMult(dualQuatConj(Mhd[:,i+1].reshape((8,1))),Mdhd[:,i+1].reshape((8,1)))
    #     A  = dualHamiltonOp(aux,0)
    #     c = -aux
    #     #prod2 = np.dot(P,Rinv)
    #     P = P -(-P@A -A.T@P + P@Rinv@P - Q)*dt
    #     #MP2[:,i] = P[:]
    #     for j in range(8):
    #         for k in range(8):
    #             MP2[j,k,i] = P[j,k]
    #     E = E - ((-1)*(A.T)@E + P@Rinv@E - P@c)*dt
    #     #for j in range(8):
    #     ME2[:,i] = E[:,0]

  
    # for i in range(0, T, 1): 
    #     #tic
    #     #Controlador LQR para O CoM
    #     #calculo de A e c
    #     #for j in range(8):
    #     mhd[:,0] = Mhd[:,i]
    #     mdhd[:,0] = Mdhd[:,i]
    #     mhd2[:,0] = Mhd2[:,i]
    #     mdhd2[:,0] = Mdhd2[:,i]
    #     aux = dualQuatMult(dualQuatConj(Mhd[:,i].reshape((8,1))),Mdhd[:,i].reshape((8,1)))
    #     A  = dualHamiltonOp(aux,0)
    #     c = -1*np.transpose(aux)
    #     #inicio do controlador  
    #     #Ja = jacobianoCinematica(theta,hOrg,hP,1,1)  
    #     xe = KinematicModel(MDH,theta,6,0)
    #     Ja = jacobiano2(theta,hOrg,hP,xe,0) 
    #     #calculo de P e E
    #     #calculo de N   
    #     Hd  = dualHamiltonOp(mhd,0)
    #     #prod3 = np.dot(Hd,C8)
    #     N  = Hd@C8@Ja
    #     #pseudo inversa de N
    #     Np  = np.linalg.pinv(N)

    #     #calculo do erro
    #     e  = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha),Mhd[:,i].reshape((8,1)))
    #     #calculo de P e E
    #     #for j in range(8):
    #     E[:,0] = ME2[:,i]
    #     #P = np.reshape(MP2[:,i],np.shape(A))
    #     # Pxe= np.dot(P,e)   
    #     # NpxRinv= np.dot(Np,Rinv)  
    #     do = Np@Rinv@(P@e + E)
    #     #calculo do o deseja
    #     od  = (do*dt)/2     
    #     for j in range(6):
    #         theta[j,0] = theta[j,0] + od[j,0]

    #     for j in range(0,6,1):
    #         if abs(theta[j,0]) > hpi:
    #             theta[j,0] = np.sign(theta[j,0])*hpi
        
 
    #     ha  = kinematicRobo(theta,hOrg,hP,1,1) 

    #     #plotar os dados
    #     for j in range(8):
    #         Mha[j,i] = ha[j,0]
    #     #posição
    #     pos = getPositionDualQuat(ha)
    #     posd = getPositionDualQuat(Mhd[:,i].reshape((8,1)))
    #     for j in range(3):
    #         Pos[j,i]  = pos[j,0]
    #         Posd[j,i] = posd[j,0]
    #     #orientação
    #     ra = getRotationDualQuat(ha)
    #     rd = getRotationDualQuat(mhd)
    #     if ra[0,0] > 1:
    #         ra[0,0] = 1
    #     co = mt.acos(ra[0,0])
    #     angle[i] = co
    #     co = mt.acos(rd[0,0])
    #     angled[i] = co
    #     for j in range(6):
    #         Mtheta[j,i] = theta[j,0]

    #     #controlador 2
    #     #calculo de A e c
    #     aux2 = dualQuatMult(dualQuatConj(mhd2),mdhd2)
    #     #A2  = dualHamiltonOp(aux2,0)
    #     c = -np.transpose(aux2)
    #     #inicio do controlador  
    #     #Ja2 = jacobianoCinematica(theta,hOrg,hP,1,0)
    #     xe2 = kinematicRobo(theta,hOrg,hP,1,0)
    #     Ja2 = jacobianoPes(theta,ha,xe2,1)
    #     #calculo de P e E
    #     #calculo de N   
    #     Hd2 = dualHamiltonOp(mhd2,0)
    #     #prod1= np.dot(Hd2,C8)
    #     N2  = Hd2@C8@Ja2
    #     #pseudo inversa de N
    #     Np2  = np.linalg.pinv(N2)
        
    #     #calculo do erro
    #     e2  = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha2),mhd2)

    #     vec2 = dualQuatMult(dualQuatConj(ha2),mdhd2)
    #     #
    #     #produto= np.dot(K2,e2-vec2)
    #     do2 = Np2@(K2@e2-vec2)
    #     od2  = (do2*dt)/2
    #     theta[:,1] = theta[:,1] + od2[:,0] 

    #     ha2 = kinematicRobo(theta,hOrg,hP,1,0)

    #     #plotar os dados
    #     #for j in range(8):
    #     Mha2[:,i] = ha2[:,0]
    #     #posição
    #     pos2 = getPositionDualQuat(ha2)
    #     posd2 = getPositionDualQuat(mhd2)
    #     for j in range(3):
    #         Pos2[j,i]  = pos2[j,0]
    #         Posd2[j,i] = posd2[j,0]
    #     #orientação
    #     ra = getRotationDualQuat(ha2)
    #     rd = getRotationDualQuat(mhd2)
    #     co = mt.acos(ra[0,0])
    #     angle2[i] = co
    #     co = mt.acos(rd[0,0])
    #     angled2[i] = co
    #     #for j in range(6):
    #     Mtheta2[:,i] = theta[:,1]

    #     #mostrar no console o andamento do metódo
    #     #msg = sprintf('#d de  #d | tempo: #f',i,T,toc)
    #     #disp(msg)
    
    # #hold on
    # plotGraficosControle(t1,dt,T,Pos,Posd,angle,angled,Mha,Mhd,Mtheta,Pos2,Posd2,angle2,angled2,Mha2,Mhd2,Mtheta2,'b','r')
    # return ha,ha2,theta,tempo, Mtheta, Mtheta2

    #global hpi, L1, L2, L3, L4, L5, height, MDH, hEdo
    glob = GlobalVariables()
    hEdo = glob.getHEDO()
    L1 = glob.getL1()
    #L2 = glob.getL2()
    #L3 = glob.getL3()
    #L4 = glob.getL4()
    #L5 = glob.getL5()
    height = glob.getHeight()
    MDH = glob.getMDH()
    hpi = glob.getHpi()
    dt = hEdo #dt é o tempo da solução da equação Edo
    hOrg = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) #posição da base 
    T = np.size(trajCoM,0)
    #T = 500;
    #t = 1:1:T;
    tempo = (T-1)*dt
    #tempo = (T)*dt
    #matrizes auxiliares
    Mhd = np.zeros((8,T))
    Mha = np.zeros((8,T))
    Mdhd = np.zeros((8,T))
    Mtheta = np.zeros((6,T))
    mhd = np.zeros((8,1))
    mdhd = np.zeros((8,1))
    mhd2 = np.zeros((8,1))
    mdhd2 = np.zeros((8,1))
    mhdPlus = np.zeros((8,1))
    mdhdPlus = np.zeros((8,1))

    Mhd2 = np.zeros((8,T))
    Mha2 = np.zeros((8,T))
    Mdhd2= np.zeros((8,T))
    Mtheta2 = np.zeros((6,T))

    angle = np.zeros(T)
    angled = np.zeros(T)
    angle2 = np.zeros(T)
    angled2 = np.zeros(T)


    Pos = np.zeros((3,T))
    Posd = np.zeros((3,T))
    Pos2 = np.zeros((3,T))
    Posd2 = np.zeros((3,T))

    #calculo de Mhd - matriz de hd
    r = np.array([1, 0, 0, 0]).reshape(4,1)
    #p = [0 0 0 0]';
    p = np.array([0, 0, -L1, -height]).reshape((4,1))
    hB1 = transformacao(p,r) #transformação base robô
    for i in range(0,T,1):
        p = np.array([0, trajCoM[i,0], trajCoM[i,1], trajCoM[i,2]]).reshape((4,1))
        r = np.array([1, 0, 0, 0]).reshape((4,1))
        hd = transformacao(p,r)
        hd = dualQuatMult(hB1,hd)
        mhd = hd
        #for j in range(8):
        Mhd[:,i] = mhd[:,0]
       
        if i <ind:
            p = np.array([0, trajPB[i,0], trajPB[i,1], trajPB[i,2]]).reshape((4,1))
            n = np.array([0, 1, 0])
            angulo = np.pi/2
            realR = np.array([mt.cos(angulo/2)])
            imagR = mt.sin(angulo/2)*n
            rb = np.concatenate((realR,imagR), axis = 0).reshape((4,1))
            hd = transformacao(p,rb)
            hd = dualQuatMult(hB1,hd)
            mhd2 = hd
            #for j in range(8):
            Mhd2[:,i] = mhd2[:,0]
        else:
            #for j in range(8):
            Mhd2[:,i] = Mhd2[:,ind-1]



    hP = ha2
   
    #Mdhd[:,0]  = (Mhd[:,0]  - Mhd[:,0])*(1/dt)
    #Mdhd2[:,0]  = (Mhd2[:,0]  - Mhd2[:,0])*(1/dt)

    for i in range(1,T,1):
        #for j in range(8):
        Mdhd[:,i] =  (Mhd[:,i] - Mhd[:,i-1])*(1/dt)
        #Mdhd[:,i] = mdhd[:,0]
        Mdhd2[:,i] =  (Mhd2[:,i]- Mhd2[:,i-1])*(1/dt)
        #Mdhd2[:,i] = mdhd2[:,0]


    ##################################
    #inicio do codigo
    #LQR
    ganhoS = vecGanho[0,0]
    ganhoQ = vecGanho[1,0]
    ganhoR = vecGanho[2,0]
    #controlador proporcional
    
    ganhoK2 = vecGanho[3,0]
    K2 = ganhoK2*np.eye(8)

    #ganho P-FF
    S = ganhoS*np.eye(8)
    Q = ganhoQ*np.eye(8)
    R = ganhoR*np.eye(8)
    Rinv = np.linalg.inv(R)
    C8 = np.diag([1, -1, -1, -1, 1, -1, -1, -1])
    #iniciar condições finais esperadas para P e E
    Pf = S
    Ef = np.array([0, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1))
    

    P = Pf
    MP2 = np.zeros((8,T))
    MP2 = np.zeros((8,8,T))
    for j in range(8):
        for k in range(8):
            MP2[j,k,T-1] = P[j,k]
    E = Ef
    ME2 = np.zeros((8,T))
    #for j in range(8):
    ME2[:,T-1] = E[:,0]

   
    for i in range(T-2,-1,-1):
        #for j in range(8):
        mhdPlus[:,0] = Mhd[:,i+1]
        mdhdPlus[:,0] = Mdhd[:,i+1]
        mhd[:,0] = Mhd[:,i]
        mdhd[:,0] = Mdhd[:,i]   
        aux = dualQuatMult(dualQuatConj(Mhd[:,i+1].reshape((8,1))),Mdhd[:,i+1].reshape((8,1)))
        A  = dualHamiltonOp(aux,0)
        c = -aux
        #prod2 = np.dot(P,Rinv)
        P = P -(-P@A -A.T@P + P@Rinv@P - Q)*dt
        #MP2[:,i] = P[:]
        for j in range(8):
            for k in range(8):
                MP2[j,k,i] = P[j,k]
        E = E - ((-1)*(A.T)@E + P@Rinv@E - P@c)*dt
        #for j in range(8):
        ME2[:,i] = E[:,0]

  
    for i in range(0, T, 1): 
        #tic
        #Controlador LQR para O CoM
        #calculo de A e c
        #for j in range(8):
        mhd[:,0] = Mhd[:,i]
        mdhd[:,0] = Mdhd[:,i]
        mhd2[:,0] = Mhd2[:,i]
        mdhd2[:,0] = Mdhd2[:,i]
        aux = dualQuatMult(dualQuatConj(Mhd[:,i].reshape((8,1))),Mdhd[:,i].reshape((8,1)))
        A  = dualHamiltonOp(aux,0)
        c = -1*np.transpose(aux)
        #inicio do controlador  
        #Ja = jacobianoCinematica(theta,hOrg,hP,1,1)  
        xe = KinematicModel(MDH,theta,6,0)
        Ja = jacobiano2(theta,hOrg,hP,xe,0) 
        #calculo de P e E
        #calculo de N   
        Hd  = dualHamiltonOp(mhd,0)
        #prod3 = np.dot(Hd,C8)
        N  = Hd@C8@Ja
        #pseudo inversa de N
        Np  = np.linalg.pinv(N)

        #calculo do erro
        e  = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha),Mhd[:,i].reshape((8,1)))
        #calculo de P e E
        #for j in range(8):
        E[:,0] = ME2[:,i]
        P[:,:] = MP2[:,:,i].reshape((8,8))
        #P = np.reshape(MP2[:,i],np.shape(A))
        # Pxe= np.dot(P,e)   
        # NpxRinv= np.dot(Np,Rinv)  
        do = Np@Rinv@(P@e + E)
        #calculo do o deseja
        od  = (do*dt)/2     
        for j in range(6):
            theta[j,0] = theta[j,0] + od[j,0]

        #for j in range(0,6,1):
            # if abs(theta[j,0]) > hpi:
            #     theta[j,0] = np.sign(theta[j,0])*hpi
        
 
        ha  = kinematicRobo(theta,hOrg,hP,1,1) 

        #plotar os dados
        for j in range(8):
            Mha[j,i] = ha[j,0]
        #posição
        pos = getPositionDualQuat(ha)
        posd = getPositionDualQuat(Mhd[:,i].reshape((8,1)))
        for j in range(3):
            Pos[j,i]  = pos[j,0]
            Posd[j,i] = posd[j,0]
        #orientação
        ra = getRotationDualQuat(ha)
        rd = getRotationDualQuat(mhd)
        co = mt.acos(ra[0,0])
        angle[i] = co
        co = mt.acos(rd[0,0])
        angled[i] = co
        for j in range(6):
            Mtheta[j,i] = theta[j,0]

        #controlador 2
        #calculo de A e c
        aux2 = dualQuatMult(dualQuatConj(mhd2),mdhd2)
        #A2  = dualHamiltonOp(aux2,0)
        c = -np.transpose(aux2)
        #inicio do controlador  
        #Ja2 = jacobianoCinematica(theta,hOrg,hP,1,0)
        xe2 = kinematicRobo(theta,hOrg,hP,1,0)
        Ja2 = jacobianoPes(theta,ha,xe2,1)
        #calculo de P e E
        #calculo de N   
        Hd2 = dualHamiltonOp(mhd2,0)
        #prod1= np.dot(Hd2,C8)
        N2  = Hd2@C8@Ja2
        #pseudo inversa de N
        Np2  = np.linalg.pinv(N2)
        
        #calculo do erro
        e2  = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha2),Mhd2[:,i].reshape((8,1)))

        vec2 = dualQuatMult(dualQuatConj(ha2),Mhd2[:,i].reshape((8,1)))
        #
        #produto= np.dot(K2,e2-vec2)
        do2 = Np2@(K2@e2-vec2)
        od2  = (do2*dt)/2
        theta[:,1] = theta[:,1] + od2[:,0] 

        ha2 = kinematicRobo(theta,hOrg,hP,1,0)

        #plotar os dados
        #for j in range(8):
        Mha2[:,i] = ha2[:,0]
        #posição
        pos2 = getPositionDualQuat(ha2)
        posd2 = getPositionDualQuat(Mhd2[:,i].reshape((8,1)))
        for j in range(3):
            Pos2[j,i]  = pos2[j,0]
            Posd2[j,i] = posd2[j,0]
        #orientação
        ra = getRotationDualQuat(ha2)
        rd = getRotationDualQuat(Mhd2[:,i].reshape((8,1)))
        co = mt.acos(ra[0,0])
        angle2[i] = co
        co = mt.acos(rd[0,0])
        angled2[i] = co
        #for j in range(6):
        Mtheta2[:,i] = theta[:,1]

        #mostrar no console o andamento do metódo
        #msg = sprintf('#d de  #d | tempo: #f',i,T,toc)
        #disp(msg)
    
    #hold on
    plotGraficosControle(t1,dt,T,Pos,Posd,angle,angled,Mha,Mhd,Mtheta,Pos2,Posd2,angle2,angled2,Mha2,Mhd2,Mtheta2,'b','r')
    return ha,ha2,theta,tempo, Mtheta, Mtheta2
