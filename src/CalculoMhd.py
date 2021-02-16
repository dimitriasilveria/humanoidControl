import numpy as np 
from globalVariables import GlobalVariables
from transformacao import transformacao
from dualQuatMult import dualQuatMult
from kinematicRobo import kinematicRobo

def calculoMhd(trajCoM,theta,trajP):

    glob = GlobalVariables()
    hEdo = glob.getHEDO()
    L1 = glob.getL1()
    height = glob.getHeight()
    #MDH = glob.getMDH()
    hpi = glob.getHpi()

    ind = np.size(trajCoM)
        
    hOrg = np.array([[1],[0], [0], [0], [0], [0], [0], [0]]) #posição da base
    dt = hEdo 
    #cacular posição atual do´pé
    n = np.array([0, 1, 0]) # n é o vetor diretor do quaternio
    thetab = hpi #parametro da função de caminhada que é igual a pi/2
    realRb = np.array([np.cos(thetab/2)])
    rb =  np.concatenate((realRb,np.sin(thetab/2)*n), axis = 0).reshape((4,1))
    pb = np.array([[0],[0], [-L1], [-height]])

    #base B para a base O6 (B é a perna em movimento)
    hB_O6 = transformacao(pb,rb)
    hP = dualQuatMult(hOrg,hB_O6)
   
	#dt é o tempo da solução da equação Edo e, ao mesmo tempo, o passo
    T = np.size(trajCoM,0) #o tamanho de trajCoM = ind

    #T = 100;
    #tempo = (T-1)*dt #o tempo é o produto da quantidade de iterações necessárias para calcular a trajetória do CoM
    tempo = (T-1)*dt
    #pelo tamanho do intervalo de tempo(passo)
    #t = 1:1:T;

    #matrizes e vetores auxiliares
    Mhd = np.zeros((8,T))
    Mha = np.zeros((8,T))
    Mdhd = np.zeros((8,T))
    Mtheta = np.zeros((6,T))
    Mhd = np.zeros((8,1))
    Mdhd = np.zeros((8,1))
    Mhd2 = np.zeros((8,1))
    Mdhd2 = np.zeros((8,1))

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
    
    #calculo de Mhd - matriz de hd
    r = np.array([1, 0, 0, 0]).reshape((4,1))
    p = np.array([0, 0, -L1, -height]).reshape((4,1))  #height = L2 + L3 + L4 + L5
    hB1 = transformacao(p,r)#transformação base robô
    for i in range(0,T,1): 
        p = np.array([0, trajCoM[i,0],trajCoM[i,1],trajCoM[i,2]]).reshape((4,1))
        r = np.array([1, 0, 0, 0]).reshape((4,1))
        hd = transformacao(p,r)
        hd = dualQuatMult(hB1,hd)
        for j in range(8):
            Mhd[j,i] = hd[j,0]
        #transformação da base até o centro de massa
        #se i<ind, o robô ainda não atingiu a posição de td, então a transformação é calculada em relação ao pé
	    #quando o robô chega na fase de TD, a transformação é calculada em relação ao CoM
        if i <ind:
            p = np.array([0, trajP[i,0], trajP[i,1],trajP[i,2]]).reshape((4,1))
            n = np.array([0, 1, 0])
            #angulo = mt.pi/2.0 #graus ou radianos????????????????????????????????????????????????????????????/
            realRb = np.array([np.cos(thetab/2)])
            rb =  np.concatenate((realRb,np.sin(thetab/2)*n), axis = 0).reshape((4,1))  
            hd = transformacao(p,rb) #posição desejada
            hd = dualQuatMult(hB1,hd)#transformação da base até o pé
            for j in range(8):
                Mhd2[j,i] = hd[j,0]
        else:
            Mhd2[:,i] = Mhd2[:,ind-1]
  

    #Mdhd[:,0]  = (Mhd[:,0]  - Mhd[:,0])*(1/dt) #não deveria ser i-1, no segundo Mhd???????????????????????????????????
    #Mdhd2[:,0]  = (Mhd2[:,0]  - Mhd2[:,0])*(1/dt)

    for i in range (1,T,1):
        Mdhd[:,i] = (Mhd[:,i] - Mhd[:,i-1])*(1/dt) #por que ele fazer isso????????????????????????????????????????????????????
        Mdhd2[:,i]  =  (Mhd2[:,i] - Mhd2[:,i-1])*(1/dt) #derivada de hd, que é a posição desejada        

    return Mhd, Mhd2, Mdhd, Mdhd2, tempo     