import numpy as np 
from gradienteFuncao import gradienteFuncao
from globalVariables import GlobalVariables

#-----------------------------------------------------------
#Método do SGD com momento utilizado para 
#otimizar a trajetória do centro de massa do modelo 3D dual SLIP
#-----------------------------------------------------------
def SGDMomento(U0,X0,vt):
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
    glob = GlobalVariables()
    ganhoAlpha = glob.getGanhoAlpha()
    gamma = glob.getGamma()
    #global ganhoAlpha, gamma
#-----------------------------------------------------------
#cálculo do gradiente da função
#-----------------------------------------------------------     
    G = gradienteFuncao(X0,U0)
    g1 = G[0,0]
    g2 = G[1,0]
    g3 = G[2,0]
    g4 = G[3,0]
#-----------------------------------------------------------
#cálculo do momento da função
#----------------------------------------------------------- 
    vt = np.zeros((4,1))
    vt[0,0] = gamma*vt[0,0] + ganhoAlpha*g1
    vt[1,0] = gamma*vt[1,0] + ganhoAlpha*g2
    vt[2,0] = gamma*vt[2,0] + ganhoAlpha*g3
    vt[3,0] = gamma*vt[3,0] + ganhoAlpha*g4
#-----------------------------------------------------------
#atualizando o valor do vetor de controle - U 
#-----------------------------------------------------------   
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