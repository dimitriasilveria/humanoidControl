import numpy as np 
import math
from kinematicRobo import kinematicRobo
from globalVariables import GlobalVariables
from getRotationDualQuat import getRotationDualQuat
#-----------------------------------------------------------
#Cálculo da derivada do Jacobiano com o método de diferenças centrais, utilizando-se 4 pontos
#----------------------------------------------------------
def DerivadaJacobiano(theta,hOrg,hP,tipo,CoM,ind):
#-----------------------------------------------------------
#variaveis globais
#----------------------------------------------------------
    glob = GlobalVariables()
    h = glob.getH()
    #MDH = glob.getMDH()
    #global h, MDH 
    thetar = np.zeros((6,1))
    thetal = np.zeros((6,1))
    #for j in range(6):
    thetar[:,0] = theta[:,0].reshape((6,1))
    thetal[:,0] = theta[:,1].reshape((6,1))

#-----------------------------------------------------------
#cálculo do primeiro ponto para 2h
#----------------------------------------------------------
    if tipo == 1:
        if CoM == 1:
            thetar[ind-1,0] = thetar[ind-1,0] + 2*h  
        else:
            thetal[ind-1,0] = thetal[ind-1,0] + 2*h 
       
    else:
        if CoM == 1:
            thetal[ind-1,0] = thetal[ind-1,0] + 2*h  
        else:
            thetar[ind-1,0] = thetar[ind-1,0] + 2*h  
    
    #calculando f(x+2h)    
    theta1 = np.concatenate((thetar, thetal),axis=1) 
    a = kinematicRobo(theta1,hOrg,hP,tipo,CoM)

 
#-----------------------------------------------------------
#cálculo do segundo ponto para h
#----------------------------------------------------------
    if tipo == 1:
        if CoM == 1:
            thetar[ind-1,0] = thetar[ind-1,0] + h  
        else:
            thetal[ind-1,0] = thetal[ind-1,0] + h 
        
    else:
        if CoM == 1:
            thetal[ind-1,0] = thetal[ind-1,0] + h  
        else:
            thetar[ind-1,0] = thetar[ind-1,0] + h  
       
    #calculando f(x+h) 
    theta2 = np.concatenate((thetar, thetal),axis=1)   
    b = kinematicRobo(theta2,hOrg,hP,tipo,CoM)

#-----------------------------------------------------------
#cálculo do terceiro ponto para -h
#----------------------------------------------------------
    if tipo == 1:
        if CoM == 1:
            thetar[ind-1,0] = thetar[ind-1,0] - h  
        else:
            thetal[ind-1,0] = thetal[ind-1,0] - h 
        
    else:
        if CoM == 1:
            thetal[ind-1,0] = thetal[ind-1,0] - h  
        else:
            thetar[ind-1,0] = thetar[ind-1,0] - h  
       
    #calculando f(x-h) 
    theta3 = np.concatenate((thetar, thetal),axis=1)   
    c = kinematicRobo(theta3,hOrg,hP,tipo,CoM)
#-----------------------------------------------------------
#cálculo do quarto ponto para -2h
#----------------------------------------------------------
    if tipo == 1:
        if CoM == 1:
            thetar[ind-1,0] = thetar[ind-1,0] - 2*h  
        else:
            thetal[ind-1,0] = thetal[ind-1,0] - 2*h 
       
    else:
        if CoM == 1:
            thetal[ind-1,0] = thetal[ind-1,0] - 2*h  
        else:
            thetar[ind-1,0] = thetar[ind-1,0] - 2*h  
    
    #calculando f(x-2h)    theta2
    theta4 = np.concatenate((thetar, thetal),axis=1) 
    d = kinematicRobo(theta4,hOrg,hP,tipo,CoM)
#-----------------------------------------------------------
#cálculo da derivada (elemento do jacobiano)
#----------------------------------------------------------
    
    q1 = (-a + 8*b  -8*c + d)* ((1/12) * h)
    

    r = getRotationDualQuat(q1)
    normq = np.linalg.norm(r)
    
    if normq != 0:
       q1 = q1/(normq)
    return q1
