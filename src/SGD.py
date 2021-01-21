import numpy as np 
from gradienteFuncao import gradienteFuncao
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#Método do gradiente descendente estocástico utilizado para 
#otimizar a trajetória do centro de massa do modelo 3D dual SLIP
#-----------------------------------------------------------
def SGD(U0,X0):
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
    glob = GlobalVariables()
    ganhoAlpha = glob.getGanhoAlpha()
    #global ganhoAlpha 
#-----------------------------------------------------------
#cálculo do gradiente da função
#-----------------------------------------------------------    
    G = gradienteFuncao(X0,U0)
    g1 = G[0,0]
    g2 = G[1,0]
    g3 = G[2,0]
    g4 = G[3,0]
#-----------------------------------------------------------
#atualizando o valor do vetor de controle - U SGD
#-----------------------------------------------------------        
    u1 = U0[0,0] - ganhoAlpha*g1
    u2 = U0[1,0] - ganhoAlpha*g2
    u3 = U0[2,0] - ganhoAlpha*g3
    u4 = U0[3,0] - ganhoAlpha*g4
    u5 = U0[4,0]
#-----------------------------------------------------------
#Valor atualizado
#-----------------------------------------------------------       
    U = np.array([[u1],[u2],[u3],[u4],[u5]])
    return U