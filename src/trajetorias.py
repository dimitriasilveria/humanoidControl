import numpy as np
from trajetoria import trajetoria
from trajetoriaPes import trajetoriaPes
from trajetoriaPesInicio import trajetoriaPesInicio

def trajetorias(U0,X0):
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
    passoAltura      = 0.07    #altura de cada passo

    #trajetoria pé B inicial
    tamTrajPeB1 = indContadoPe
    trajPB1 = trajetoriaPes(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,1,tamTrajPeB1)
    #trajPB1 = trajetoriaPesInicio(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,tamTrajPeB1)
    
    
    passoComprimento2 = PB[0,0] #tamanho do passo
    passoLargura2     = 0 #Largura do passo
    passoAltura2      = 0.07    #altura de cada passo
    
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


    # trajCoM2[:,0] = trajCoM2[:,0] + i*2*passoTrajCoM
    # trajPA[:,0] = trajPA[:,0]+ i*2*passoComprimento

    # trajCoM3
    # #for j in range(np.size(trajCoM,0)):
    # trajCoM3[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
    # trajP = trajPB
    # #k = np.size(trajP,0)
    # #for j in range(k):
    # trajP[:,0] = trajP[:,0]+ i*2*passoComprimento

    return trajPB1, trajPB, trajPA, trajCoM1, trajCoM2, trajCoM3, passoTrajCoM, passoComprimento
