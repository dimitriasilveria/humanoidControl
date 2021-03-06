import numpy as np 
from dualQuatMult import dualQuatMult
from kinematicModel import KinematicModel
from globalVariables import GlobalVariables
from dualQuatDH import dualQuatDH
def jacobiano2(theta,hOrg,hP,xe,leg):
#-----------------------------------------------------------
#c�lculo das derivadas para cada vari�vel de controle
#----------------------------------------------------------
    z = np.zeros((8,1))
    thetar = theta[:,0].reshape((6,1))
    thetal = theta[:,1].reshape((6,1))
    glob = GlobalVariables()
    L1 = glob.getL1()
    L2 = glob.getL2()
    hpi = glob.getHpi()
    MDH = glob.getMDH()
    #hCoM_O0_rightLeg = dualQuatDH(hpi,-L2,-L1,0,0) #transformação do sist de coordenadas do centro de massa para a origem 0 da perna direita
    #hCoM_O0_leftLeg = dualQuatDH(hpi,-L2, L1,0,0)  #transformação do sist de coordenadas do centro de massa para a origem 0 da perna esquerda

    #perna = 1 (perna direita)
    #perna = 0 (perna esquerda)
    #hB = dualQuatMult(hOrg,hP)
    #hB_O6a = dualQuatMult(hOrg,hP) 
    #hB = [1 0 0 0 0 0 0 0]'
    ##################################j1##################################
    #h = dualQuatMult(hB,KinematicModel(MDH,thetar,6,0))
    if leg == 0: #right leg
        h = dualQuatMult(hP,KinematicModel(MDH,thetar,6,0)) #da base global até a junta 6
        #h = [1 0 0 0 0 0 0 0]'
        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j0 = dualQuatMult(z,xe)

        ##################################j1##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetar,5,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j1 = dualQuatMult(z,xe)


        ##################################j2##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetar,4,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j2 = dualQuatMult(z,xe)

            ##################################j3##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetar,3,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j3 = dualQuatMult(z,xe)

        ##################################j4##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetar,2,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j4 = dualQuatMult(z,xe)

        ##################################j5##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetar,1,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j5 = dualQuatMult(z,xe)
    else:
        h = dualQuatMult(hP,KinematicModel(MDH,thetal,6,0)) #da base global até a junta 6
        #h = [1 0 0 0 0 0 0 0]'
        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j0 = dualQuatMult(z,xe)

        ##################################j1##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetal,5,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j1 = dualQuatMult(z,xe)


        ##################################j2##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetal,4,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j2 = dualQuatMult(z,xe)

            ##################################j3##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetal,3,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j3 = dualQuatMult(z,xe)

        ##################################j4##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetal,2,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j4 = dualQuatMult(z,xe)

        ##################################j5##################################
        h = dualQuatMult(hP,KinematicModel(MDH,thetal,1,0))

        #z[0,0] = 0
        z[1,0] = h[1,0]*h[3,0] + h[0,0]*h[2,0]
        z[2,0] = h[2,0]*h[3,0] - h[0,0]*h[1,0]
        z[3,0] = (h[3,0]**2 - h[2,0]**2 - h[1,0]**2 + h[0,0]**2)/2 
        #z[4,0] = 0
        z[5,0] = h[1,0]*h[7,0] + h[5,0]*h[3,0] + h[0,0]*h[6,0] + h[4,0]*h[2,0]
        z[6,0] = h[2,0]*h[7,0] + h[6,0]*h[3,0] - h[0,0]*h[5,0] - h[4,0]*h[1,0]
        z[7,0] = h[3,0]*h[7,0] - h[2,0]*h[6,0] - h[1,0]*h[5,0] + h[0,0]*h[4,0]

        j5 = dualQuatMult(z,xe)


    jac = np.concatenate((-j5, -j4, -j3, -j2, -j1, -j0),axis=1)




    
    return jac