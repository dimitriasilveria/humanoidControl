import numpy as np 
import math
from kinematicRobo import kinematicRobo
from globalVariables import GlobalVariables
from getRotationDualQuat import getRotationDualQuat
from rungekutta42 import rungeKutta42
#-----------------------------------------------------------
#Cálculo da derivada de f com o método de diferenças centrais, utilizando-se 4 pontos
#u é a variável independente, em relação a qual se deseja derivar     #U = [phi, theta, k] (model1)
                                                                      #U = [phi,theta,Bss] (model2)  
#UAnterior é o produto de todos os u ou U de 0 até n-1
#An = A^n
#----------------------------------------------------------
def DerivadaFU(u,x,ind,t,model,pfb,An):
#-----------------------------------------------------------
#variaveis globais
#----------------------------------------------------------
    glob = GlobalVariables()
    h = glob.getH()
    m = glob.getM()
    L = glob.getL()
    g = glob.getG()
    k = glob.getK()
    Bss = glob.getBss()
    expK = glob.getExpK()
#------------------------------------------------------------
    #X = [x,y,z,dx,dy]
    #U = [phi, theta, k] (model1)
    #U = [phi,theta,Bss] (model2)    
    var = np.array([[t],[h],[model]])

    if model == 1:
#-----------------------------------------------------------
#cálculo do primeiro ponto para 2h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] + 2*h  
     
#calculando f(x+2h)
#xn vem na forma x = [x,y,z,dx,dy,dz], mas para ser utilizado na função
#rungeKutta42, é preciso estar na forma x = [x,dx,y,dy,z,dz]
        params = np.array([[m],[L],[g],[k],[u[2]],[expK]]) 
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        a = np.dot(An,xn)
        #a = a[:-1]
#-----------------------------------------------------------
#cálculo do primeiro ponto para h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] + h  
     
#calculando f(x+h)
        params = np.array([[m],[L],[g],[k],[u[2]],[expK]])    
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        b = np.dot(An,xn)
        #a = a[:-1]
#-----------------------------------------------------------
#cálculo do primeiro ponto para -h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] - h  
     
#calculando f(x-h)
        params = np.array([[m],[L],[g],[k],[u[2]],[expK]])    
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        c = np.dot(An,xn)
        #a = a[:-1]
#-----------------------------------------------------------
#cálculo do primeiro ponto para -2h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] - 2*h  
     
#calculando f(x-2h)
        params = np.array([[m],[L],[g],[k],[u[2]],[expK]])    
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        d = np.dot(An,xn)
        #a = a[:-1]
    else: 
        #-----------------------------------------------------------
#cálculo do primeiro ponto para 2h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] + 2*h  
     
#calculando f(x+2h)
        params = np.array([[m],[L],[g],[u[2]],[Bss],[t],[pfb[0,0]],[pfb[1,0]],[pfb[2,0]],[expK]])    
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        a = np.dot(An,xn)
        #a = a[:-1]
#-----------------------------------------------------------
#cálculo do primeiro ponto para h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] + h  
     
#calculando f(x+h)
        params = np.array([[m],[L],[g],[u[2]],[Bss],[t],[pfb[0,0]],[pfb[1,0]],[pfb[2,0]],[expK]])    
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        b = np.dot(An,xn)
        #a = a[:-1]
#-----------------------------------------------------------
#cálculo do primeiro ponto para -h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] - h  
     
#calculando f(x-h)
        params = np.array([[m],[L],[g],[u[2]],[Bss],[t],[pfb[0,0]],[pfb[1,0]],[pfb[2,0]],[expK]])   
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        c = np.dot(An,xn)
        #a = a[:-1]
#-----------------------------------------------------------
#cálculo do primeiro ponto para -2h
#----------------------------------------------------------
        u[ind,0] = u[ind,0] - 2*h  
     
#calculando f(x-2h)
        params = np.array([[m],[L],[g],[k],[u[2]],[expK]])    
        yn = baguncaX(x)   
        xn = rungeKutta42(var,yn,params)
        #voltar xn para a forma xn = [x,y,z,dx,dy,dz] e eliminar o último elemento
        xn = arrumaX(xn)
        xn = xn[0:5,:]
        d = np.dot(An,xn)
        #a = a[:-1]

    q1 = (-a + 8*b  -8*c + d)* ((1/12) * h)

    return q1

def baguncaX(X):
    #x = [x,y,z,dx,dy,dz]
    #xEdo = [x,dx,y,dy,z,dz]
    x = np.array([X[0,0],X[3,0],X[1,0],X[4,0],X[2,0],X[5,0]]).reshape(6,1)
    return x 

def arrumaX(X):
    x = np.array([X[0,0],X[2,0],X[4,0],X[1,0],X[3,0],X[5,0]]).reshape(6,1)
    return x