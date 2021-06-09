from LIPM import LIPM
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
def rungeKuttaLIPM(var,y,params):
#-----------------------------------------------------------
#parâmetros para o método de rungeKutta
#-----------------------------------------------------------
    t   = var[0,0]
    h   = var[1,0]
    yk = np.zeros((6,1))
#-----------------------------------------------------------
#verifica qual o modelo será resolvido
# 1 - modelo 1
# != 1 - modelo 2
#-----------------------------------------------------------   

#-----------------------------------------------------------
#cálculo dos parâmetros do método de rungeKutta
#-----------------------------------------------------------
    K1 = LIPM(t,y,params)
    K2 = LIPM(t + h/2,y + (h/2)*K1,params)
    K3 = LIPM(t + h/2,y + (h/2)*K2,params)
    K4 = LIPM(t + h,y + h*K3,params)
#-----------------------------------------------------------
#novo valor de yk e o novo passo
#-----------------------------------------------------------
    yk = y + (h/6)*(K1 + 2*K2 + 2*K3 + K4)
    yk[4,0] = y[4,0]
    yk[5,0] = y[5,0]


    return yk

    