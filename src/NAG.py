import numpy as np 
from gradienteFuncao import gradienteFuncao
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#Método do Nesterov accelerated gradient utilizado para 
#otimizar a trajetória do centro de massa do modelo 3D dual SLIP
#-----------------------------------------------------------
def NAG(U0,X0,vt):
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
    glob = GlobalVariables()
    ganhoAlpha = glob.getGanhoAlpha()
    gamma = glob.getGamma()
    #global ganhoAlpha, gamma
#-----------------------------------------------------------
#cálculando o valor com o ganho anterior
#-----------------------------------------------------------    
    y1 = U0[0,0] - gamma*vt[0,0]
    y2 = U0[1,0] - gamma*vt[1,0]
    y3 = U0[2,0] - gamma*vt[2,0]
    y4 = U0[3,0] - gamma*vt[3,0]
    y5 = U0[4,0]    
    Y0 = np.array([[y1], [y2], [y3], [y4], [y5]])
#-----------------------------------------------------------
#cálculo do gradiente da função
#-----------------------------------------------------------       
    G = gradienteFuncao(X0,Y0)
    g1 = G[0,0]
    g2 = G[1,0]
    g3 = G[2,0]
    g4 = G[3,0]
#-----------------------------------------------------------
#cálculando o valor com o ganho atual
#-----------------------------------------------------------      
    vt[0,0] = gamma*vt[0,0] + ganhoAlpha*g1
    vt[1,0] = gamma*vt[1,0] + ganhoAlpha*g2
    vt[2,0] = gamma*vt[2,0] + ganhoAlpha*g3
    vt[3,0] = gamma*vt[3,0] + ganhoAlpha*g4

    u1 = U0[0,0] - vt[0,0]
    u2 = U0[1,0] - vt[1,0]
    u3 = U0[2,0] - vt[2,0]
    u4 = U0[3,0] - vt[3,0]
    u5 = U0[4,0]
#-----------------------------------------------------------
#Valor atualizado
#-----------------------------------------------------------         
    U = np.array([[u1],[u2],[u3],[u4],[u5]])
    return U, vt