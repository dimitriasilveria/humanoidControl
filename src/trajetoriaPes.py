import numpy as np
#---------------------------------------
#Calcular a trajetória dos pés
#Função de Bézier de 4ª ordem
#---------------------------------------
def trajetoriaPes(posP,passo,altura,metade,tam):
    
    p0 = np.array([[posP[0,0]],[posP[2,0]]]) + np.array([[-2*passo],[0]]) 
    p1 = np.array([[posP[0,0]],[posP[2,0]]]) + np.array([[-passo],[altura]]) 
    p2 = np.array([[posP[0,0]],[posP[2,0]]]) 
    
    dt = 1/(2*tam-1)
    t = np.array(np.arange(0,1+dt,dt)) 
    t = t.reshape(1,-1)
    ind = np.size(t,axis=1)
    k = ind-tam
    B = np.zeros((ind,2))
    for i in range(0,ind,1):
        B[i,0] = (1 - t[0,i])**2 * p0[0,0] + 2*t[0,i]*(1 - t[0,i])*p1[0,0] + t[0,i]*t[0,i]*p2[0,0]
        B[i,1] = (1 - t[0,i])**2 * p0[1,0] + 2*t[0,i]*(1 - t[0,i])*p1[1,0] + t[0,i]*t[0,i]*p2[1,0]
   
    
    if metade ==1:
        trajPes = np.ones((k,3))
        for j in range(k):
            trajPes[j,0] = B[j+tam,0]
            trajPes[j,1] = posP[1,0]
            trajPes[j,2] = B[j+tam,1]
        
        #trajPes = np.transpose(trajPes)
    else:
        trajPes = np.ones((ind,3))
        for j in range(ind):
            trajPes[j,0] = B[j,0]
            trajPes[j,1] = posP[1,0]
            trajPes[j,2] = B[j,1]  
        #trajPes = np.transpose(trajPes)
    return trajPes

