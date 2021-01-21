import numpy as np 
from primeiraDerivadaGradiente import primeiraDerivadaGradiente
#-----------------------------------------------------------
#Calcula o gradiente da função objetivo
#Parâmetros :
#X0 - Vetor com a condição inicial do sistema X (Constante)
#U0 - Vetor com a condição inicial das variáveis de controle u
#Retorno:
#grad -  vetor gradiente das variáveis de controle
#----------------------------------------------------------
def gradienteFuncao(X0,U0):
#-----------------------------------------------------------
#cálculo do gradiente para cada variável de controle
#----------------------------------------------------------    
    grad = np.zeros((4,1))
    grad[0,0] = primeiraDerivadaGradiente(X0,U0,1)# linha referente a theta
    grad[1,0] = primeiraDerivadaGradiente(X0,U0,2)# linha referente a phi
    grad[2,0] = primeiraDerivadaGradiente(X0,U0,3)# linha referente a K
    grad[3,0] = primeiraDerivadaGradiente(X0,U0,4)# linha refetente a BSS   
    return grad