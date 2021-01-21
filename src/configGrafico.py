#---------------------------
#Configurar Gráficos
#--------------------------
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D as ax
def configGrafico():
    fig = plt.figure(figsize=(60,35))
    ax = fig.add_subplot(111,projection='3d')
    plt.grid(True)
    ax.set_xlim([-1,1])
    ax.set_ylim([-0.5,0.5])
    ax.set_zlim([-1,-0.2])
    #axis([-1 1 -0.5 0.5 -1 0.2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')


