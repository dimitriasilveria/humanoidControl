#--------------------------------------
#Método para obter a rotação de um quatérnio dual
#Parâmetros: 
#q: quatérnio dual
#Retorno:
#r: quatérnio de rotação (orientação)
#--------------------------------------
import numpy as np
from quatScale import quatScale
from quatNorm import quatNorm
def getRotationDualQuat(q):
     normq = quatNorm(q)
        
     if normq != 0:
       q1 = q1/(normq)

     r = np.array([q1[0,0], q1[1,0], q1[2,0], q1[3,0]]).reshape((4,1))
     return r

