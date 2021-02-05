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
     if normq == 0:
          qr = q
     else:
          qr = quatScale(1.0 / normq, q)

     r = np.array([qr[0,0], qr[1,0], qr[2,0], qr[3,0]]).reshape((4,1))
     return r

