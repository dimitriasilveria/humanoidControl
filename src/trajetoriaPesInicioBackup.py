import numpy as np
from plotarPes import plotarPes
#---------------------------------------
#Calcular a trajetória dos pés
#Função de Bézier de 4ª ordem
#---------------------------------------
def trajetoriaPesInicio2(posP,passo,altura,tam):
    
    # p0 = np.array([[posP[0,0]],[posP[2,0]]]) + np.array([[-passo],[0]])
    # p1 = np.array([[posP[0,0]],[posP[2,0]]]) + np.array([[-passo/2],[altura/3]])
    # p2 = np.array([[posP[0,0]],[posP[2,0]]]) 

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
   
    #trajPes = np.transpose(trajPes)

    # trajPes = np.ones((ind,3))
     
    # for j in range(ind):
    #     trajPes[j,0] = B[j,0]
    #     trajPes[j,1] = posP[1,0]
    #     trajPes[j,2] = B[j,1]  

    # #trajPes = np.transpose(trajPes)
    trajPes = np.zeros((ind,3))
    step = np.max(B[:,1])/k
    max = float(np.max(B[:,1]))
    height = np.arange(0,max,step)
    print(height[6])
    
    
    #trajPes = [B[tam+1:ind,1],posP[2,1]*np.ones((tam,1)),B[tam+1:ind,2]]
    for j in range(k):
        trajPes[j,1] = posP[1,0]
        trajPes[j,2] = height[j]
        trajPes[tam+j,0] = B[j+tam,0]
        trajPes[tam+j,1] = posP[1,0]
        trajPes[tam+j,2] = B[j+tam,1]

    plotarPes(trajPes[:,0],trajPes[:,2])
    return trajPes
