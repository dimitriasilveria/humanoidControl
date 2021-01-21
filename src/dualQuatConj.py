#--------------------------------------
#Método para calcular o quatérnio dual conjugado
#Parâmetros: 
#q: quatérnio dual
#Retorno:
#qr: quatérnio dual conjugado
#--------------------------------------
from quatConj import quatConj
import numpy as np
def dualQuatConj(q):
    a = np.array([q[0,0],q[1,0],q[2,0],q[3,0]]).reshape((4,1))
    b = np.array([q[4,0],q[5,0],q[6,0],q[7,0]]).reshape((4,1))
    qp = quatConj(a)
    qd = quatConj(b)
    #qp =  quatConj([q(1,1) q(2,1) q(3,1) q(4,1)]')
    #qd =  quatConj([q(5,1) q(6,1) q(7,1) q(8,1)]')
    qr = np.concatenate((qp,qd),axis=0) 
    return qr     
