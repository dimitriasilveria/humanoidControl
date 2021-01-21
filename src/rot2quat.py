#--------------------------------------
#Método para calcular o quatérnio equivalente
#dado uma matriz de rotação 
#Parâmetros: 
#T: Matriz de Rotação
#Retorno:
#q: quatérnio resultante
#--------------------------------------
import numpy as np
import math as mt 
def rot2quat(T):
    
    #iniciar o quarténio
    q = np.array([[0], [0], [0], [0]])

    #Elementos da matriz de Rotação
    D1 = T[0,0]
    D2 = T[1,1]
    D3 = T[2,2]

    Au = T[0,1]
    Bu = T[0,2]
    Cu = T[1,2]

    Ad = T[1,0]
    Bd = T[2,0]
    Cd = T[2,0]

    #solução baseada em w
    bw = 1 + D1 + D2 + D3
    if bw > 0:
        print('w')
        w = sqrt(bw)/2
        x = (Cd - Cu)/(4*w)
        y = (Bu - Bd)/(4*w)
        z = (Ad - Au)/(4*w)

        q = np.array([[w], [x], [y], [z]])
        return
    

    #solução baseada em x
    if D1 > D2 & D1 > D3:
        print('x')
        bx = 1 + D1 - D2 -D3
        x = mt.sqrt(bx)/2
        y = (Au+Ad)/(4*x)
        z = (Bu+Bd)/(4*x)
        w = (Cd - Cu)/(4*x)

        q = np.array([[w], [x], [y], [z]])
        return
   

    #solução baseada em y
    if D2 > D1 & D2 > D3:
        print('y')
        by = 1 - D1 + D2 -D3
        y = mt.sqrt(by)/2
        x = (Au+Ad)/(4*y)
        z = (Cd+Cu)/(4*y)
        w = (Bu - Bd)/(4*y)

        q = np.array([[w], [x], [y], [z]])
        return
    

    #solução baseada em z
    if D3 > D1 & D3 > D2:
        print('z')
        bz = 1 - D1 - D2 +D3
        z = mt.sqrt(bz)/2
        x = (Bu+Bd)/(4*z)
        y = (Cd+Cu)/(4*z)
        w = (Ad - Au)/(4*z)

        q = np.array([[w], [x], [y], [z]])
        return
    
    return q