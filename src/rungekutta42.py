from model1 import model1
from model2 import model2
import numpy as np 
#-----------------------------------------------------------
#método de RungeKutta de 4ª ordem
#para a resolução numérica (aproximação) 
#Soluções de equações diferenciais ordinárias.
#Parâmetros:
#var - vetor com parâmetros para o método
#params - vetor com os parâmetros passados para o modelo dinâmico
#Retorno:
#yk - nova condição inicial , solução para a EDO para a iteração
#sh - novo passo ótimo - (nesse caso esta passando o passo fixo, sempre h)
#-----------------------------------------------------------
def rungeKutta42(var,y,params):
#-----------------------------------------------------------
#parâmetros para o método de rungeKutta
#-----------------------------------------------------------
    t   = var[0,0]
    h   = var[1,0]
    fun = var[2,0]
    yk = np.zeros((6,1))
#-----------------------------------------------------------
#verifica qual o modelo será resolvido
# 1 - modelo 1
# != 1 - modelo 2
#-----------------------------------------------------------   
    if fun==1: 
#-----------------------------------------------------------
#cálculo dos parâmetros do método de rungeKutta
#-----------------------------------------------------------
        K1 = model1(t,y,params)
        K2 = model1(t + h/2,y + (h/2)*K1,params)
        K3 = model1(t + h/2,y + (h/2)*K2,params)
        K4 = model1(t + h,y + h*K3,params)
#-----------------------------------------------------------
#novo valor de yk e o novo passo
#-----------------------------------------------------------
        yk = y + (h/6)*(K1 + 2*K2 + 2*K3 + K4)
    else:
#-----------------------------------------------------------
#cálculo dos parâmetros do método de rungeKutta
#-----------------------------------------------------------
        K1 = model2(t,y,params)
        K2 = model2(t + h/2,y + (h/2)*K1,params)
        K3 = model2(t + h/2,y + (h/2)*K2,params)
        K4 = model2(t + h,y + h*K3,params)
#-----------------------------------------------------------
#novo valor de yk e o novo passo
#-----------------------------------------------------------
        yk = y + (h/6)*(K1 + 2*K2 + 2*K3 + K4)
    return yk

    