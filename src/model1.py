import numpy as np
import math as mt 
#-----------------------------------------------------------
#Calcula o valor das derivadas do modelo dinâmico 1
#Parâmetros
#t   - tempo
#Y0  - Vetor com a condição inicial do sistema
#params: parâmetros do sistema
#m   - massa
#L   - comprimento da perna do modelo
#g   - gravidade
#k   - constante da mola
#Bss - coeficiente angular do modelo
#Retorno:
#dydt - vetor com primeira e segunda derivada do sistema
#-----------------------------------------------------------
def model1(t,Y0,params):

#-----------------------------------------------------------
#constantes
#-----------------------------------------------------------
    m    = params[0,0]
    L    = params[1,0]
    g    = params[2,0]
    k    = params[3,0]
    Bss  = params[4,0]
    expK = params[5,0]    
#-----------------------------------------------------------
#condição inicial
#----------------------------------------------------------- 
    #Y0 = np.array((6,1))
    x  = Y0[0,0]
    dx = Y0[1,0]
    y  = Y0[2,0]
    dy = Y0[3,0]
    z  = Y0[4,0]
    dz = Y0[5,0]
#-----------------------------------------------------------
#norma de L
#-----------------------------------------------------------      
    norma = (x*x + y*y + z*z )**0.5
#-----------------------------------------------------------
#derivadas
#-----------------------------------------------------------   
    f1 = dx
    f2 = (k*expK/m)*( L +Bss*t - norma)*(x/norma)
    f3 = dy
    f4 = (k*expK/m)*( L +Bss*t - norma)*(y/norma)
    f5 = dz
    f6 = (k*expK/m)*( L +Bss*t - norma)*(z/norma) - g
#-----------------------------------------------------------
#solução
#-----------------------------------------------------------    
    dydt = np.array([[f1],[f2],[f3],[f4],[f5],[f6]])
    return dydt