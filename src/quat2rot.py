import numpy as np
#--------------------------------------
#Método para calcular a matriz de rotação
#equivalente dado um quatérnio
#Parâmetros: 
#q: quatérnio
#Retorno:
#R: matriz de rotação
#--------------------------------------
def quat2rot(q):   
    #inicializar variáveis
    w = q[0,0]
    x = q[1,0]
    y = q[2,0]
    z = q[3,0]  
    #montar matriz de rotação
    R = np.zeros((4,4))
    R[0,0] = w*w + x*x - y*y - z*z
    R[1,0] = 2*x*y + 2*z*w
    R[2,0] = 2*x*z - 2*y*w
    R[3,0] = 0
    R[0,1] = 2*x*y - 2*z*w
    R[1,1] = w*w - x*x + y*y - z*z
    R[2,1] = 2*y*z + 2*x*w
    R[3,1] = 0
    R[0,2] = 2*x*z + 2*y*w
    R[1,2] = 2*y*z - 2*x*w
    R[2,2] = w*w - x*x - y*y + z*z
    R[3,2] = 0
    R[0,3] = 0
    R[1,3] = 0
    R[2,3] = 0
    R[3,3] = 1   
    return R