#---------------------------------------
#Método para plotar o sistema de coordenadas
#---------------------------------------
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D as ax
import DualQuaternionClass
import transformacao
def plotCoord(hOrg):
    Org = hOrg.getOrientationVector()
    hX = hOrg.mult.transformacao([0,0.1,0,0],[1,0,0,0])
    hY = hOrg.mult.transformacao([0,0,0.1,0],[1,0,0,0])
    hZ = hOrg.mult.transformacao([0,0,0,0.1],[1,0,0,0])

    plotVector1 = plt.figure(Org,hX.getOrientationVector(),'b')
    plotVector2 = plt.figure(Org,hY.getOrientationVector(),'r')
    plotVector3 = plt.figure(Org,hZ.getOrientationVector(),'g')
    plt.show()
