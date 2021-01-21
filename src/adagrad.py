import numpy as np 
import math as mt
from gradienteFuncao import gradienteFuncao
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#Método do adagrad utilizado para 
#otimizar a trajetória do centro de massa do modelo 3D dual SLIP
#-----------------------------------------------------------
def adagrad(U0,X0,G0):
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
    glob = GlobalVariables()
    ganhoAlpha = glob.getGanhoAlpha()
    #X0 = np.array((6,1))
    #global ganhoAlpha 
    
    h = 10**(-8) 
#-----------------------------------------------------------
#valor do ganho anterior
#-----------------------------------------------------------
    G1 = G0[0,0]
    G2 = G0[1,0]
    G3 = G0[2,0]
    G4 = G0[3,0]
#-----------------------------------------------------------
#cálculo do gradiente da função
#-----------------------------------------------------------    
    G = gradienteFuncao(X0,U0)
    g1 = G[0,0]
    g2 = G[1,0]
    g3 = G[2,0]
    g4 = G[3,0]
#-----------------------------------------------------------
#cálculo do novo ganho
#-----------------------------------------------------------      
    G1 = G1 + g1*g1
    G2 = G2 + g2*g2
    G3 = G3 + g3*g3
    G4 = G4 + g4*g4
    G0 = np.array([[G1],[G2],[G3],[G4]])
#-----------------------------------------------------------
#atualizando o valor do vetor de controle - U 
#-----------------------------------------------------------     
    u1 = U0[0,0] - (ganhoAlpha/(mt.sqrt(G1+h)))*g1
    u2 = U0[1,0] - (ganhoAlpha/(mt.sqrt(G2+h)))*g2
    u3 = U0[2,0] - (ganhoAlpha/(mt.sqrt(G3+h)))*g3
    u4 = U0[3,0] - (ganhoAlpha/(mt.sqrt(G4+h)))*g4
    u5 = U0[4,0]
#-----------------------------------------------------------
#Valor atualizado
#-----------------------------------------------------------        
    
    U = np.array([[u1],[u2],[u3],[u4],[u5]])
    return U, G0 