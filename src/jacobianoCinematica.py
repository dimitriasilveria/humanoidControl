import numpy as np 
import math
from DerivadaJacobiano import DerivadaJacobiano
#-----------------------------------------------------------
#Cálculo do Jacobiano Analítico
#Parâmetros:
#theta - vetor com as variáveis de juntas 
#hOrg  - quatérnio dual com a origem do sistema de referência
#hP    - quatérnio dual com a posição do pé de apoio
#tipo  - flag que indica se a cinemática é da perna direita ou esquerda
#CoM   - flag para indicar se é para computar a cinemática até o centro de 
#		 massa ou até o outro pé.
#----------------------------------------------------------   
def jacobianoCinematica(theta,hOrg,hP,tipo,CoM):
#-----------------------------------------------------------
#cálculo das derivadas para cada variável de controle
#----------------------------------------------------------    
    J = np.zeros((8,6))
    J1= DerivadaJacobiano(theta,hOrg,hP,tipo,CoM,1)
    J2= DerivadaJacobiano(theta,hOrg,hP,tipo,CoM,2)
    J3= DerivadaJacobiano(theta,hOrg,hP,tipo,CoM,3)
    J4= DerivadaJacobiano(theta,hOrg,hP,tipo,CoM,4)
    J5= DerivadaJacobiano(theta,hOrg,hP,tipo,CoM,5)
    J6= DerivadaJacobiano(theta,hOrg,hP,tipo,CoM,6)
    
    for j in range(8):
        J[j,0] = J1[j,0]
        J[j,1] = J2[j,0]
        J[j,2] = J3[j,0]
        J[j,3] = J4[j,0]
        J[j,4] = J5[j,0]
        J[j,5] = J6[j,0] 

    return J