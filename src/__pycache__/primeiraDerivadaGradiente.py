import numpy as np 
import math
import cmath
from trajetoria import trajetoria
from funcaoObjetivo import funcaoObjetivo
#-----------------------------------------------------------
#Cálculo da derivada da função objetivo dado a variação de 
#um parâmetro
#modelo dual SLIP melhorado
#derivada calculada pelo método das diferenças centradas
#utilizando 4 pontos, este método tem uma incerteza
#da O(h^4)
#Parâmetros :
#X0 - Vetor com a condição inicial do sistema (Constante)
#U0 - Vetor com a condição inicial das variáveis de controle u
#Retorno:
#d1 -  derivada
#----------------------------------------------------------
def primeiraDerivadaGradiente(X0,U0,ind):
#-----------------------------------------------------------
#variáveis globais
#----------------------------------------------------------
    global h
#-----------------------------------------------------------
#cálculo do primeiro ponto para 2h
#----------------------------------------------------------
    y = U0[ind,0]+2*h
    Y = U0[:,0] #copiando o vetor de controle original
    Y[ind,0] = y #substituindo o valor com a variação na posição desejada
    [pa,pb,pc,M,ponto] = trajetoria(Y,X0,ind) #cálculo dos pontos da trajetória
    #pa - ponto do pé A
    #pb - ponto do pé B
    #pc - ponto do centro de massa
    a = funcaoObjetivo(pa,pb,pc) #calculo da função objetivo
#-----------------------------------------------------------
#cálculo do segundo ponto para h
#----------------------------------------------------------
    y = U0[ind,0]+h
    Y = U0[:,0]
    Y[ind,0] = y
    [pa,pb,pc,M,ponto] = trajetoria(Y,X0,ind)
    b = funcaoObjetivo(pa,pb,pc)
#-----------------------------------------------------------
#cálculo do terceiro ponto para -h
#----------------------------------------------------------
    y = U0[ind,0]-h
    Y = U0[:,0]
    Y[ind,0] = y
    [pa,pb,pc,M,ponto] = trajetoria(Y,X0,ind) #tive de acrescentar as variáveis M e ponto, pois a função trajetória 
    #retorna todos esses valores, mas só me interessam pa, pb e pc, porém, não sei como pegar, no python, 
    #apenas uma parte do retorno da função
    c = funcaoObjetivo(pa,pb,pc)
#-----------------------------------------------------------
#cálculo do quarto ponto para -2h
#----------------------------------------------------------
    y = U0[ind,0] - 2*h
    Y = U0[:,0]
    Y[ind,0] = y
    [pa,pb,pc,M,ponto] = trajetoria(Y,X0,ind)
    d = funcaoObjetivo(pa,pb,pc)
#-----------------------------------------------------------
#cálculo da derivada
#----------------------------------------------------------
    d1 = (-a + 8*b  -8*c + d) / (12 * h)  + h^4
    return d1