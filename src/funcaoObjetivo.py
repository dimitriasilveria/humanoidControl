import numpy as np 
import math as mt 
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#Calcula o valor da função objetivo
#função objetivo = função custo
#Parâmetros: 
#pa - Vetor com posição espacial do pé A
#pb - Vetor com posição espacial do pé B
#pc - Vetor com posição espacial do centro de massa
#Retorno: 
#fo -  valor da função objetivo
#-----------------------------------------------------------
def funcaoObjetivo(pa,pb,pc):
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
    #global lstep
#-----------------------------------------------------------
#Pegar os valores de x e y do pé A e B e do CoM
#CoM - centro de massa
#-----------------------------------------------------------
    xa = float(pa[0,0]) #posição x do pé A
    xb = float(pb[0,0]) #posição x do pé B
    ya = float(pa[1,0]) #posição y do pé A
    yb = float(pb[1,0]) #posição x do pé B
    xc = float(pc[0,0]) #posição x do CoM
    yc = float(pc[1,0]) #posição x do CoM
#-----------------------------------------------------------
#cálcula a norma euclidiano do espaço (função objetivo - fo)
#-----------------------------------------------------------
    s = np.zeros((2,1))
    s[0,0] = (0.5*(xa + xb) - xc)
    s[1,0] = (0.5*(ya + yb) - yc)
    fo = s[0,0]**2 + s[1,0]**2
    return fo