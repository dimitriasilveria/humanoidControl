import numpy as np
import math as mt
from rungekutta42 import rungeKutta42 
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D as ax
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#Método para mostrar graficamente a trajetória do modelo dinâmico
#Parâmetros:
#U - variáveis de controle
#X - vetor com as condições iniciais
#Retorno:
#gráfico do modelo dinâmico
#-----------------------------------------------------------
def plotarTrajetoria(U,X):
#-----------------------------------------------------------    
#variáveis globais
#-----------------------------------------------------------
#global m L g pfa expK hEdo
    glob = GlobalVariables()
    m = glob.getM()
    L = glob.getL()
    g = glob.getG()
    pfa = glob.getPfa()
    hEdo = glob.getHEDO()
    expK   = glob.getExpK()
#-----------------------------------------------------------
#condições iniciais para MS
#----------------------------------------------------------- 
    xod  = X[0,0]
    yod  = X[1,0]
    zod  = X[2,0]
    dxod = X[3,0]
    dyod = X[4,0]
    dzod = X[5,0]
#-----------------------------------------------------------
#valores para a otimização - valores de u
#-----------------------------------------------------------  
    #u = [phi theta k Bss]
    theta = U[0,0]
    phi   = U[1,0]
    k     = U[2,0]
    Bss   = U[3,0]
    expK  = U[4,0]
#-----------------------------------------------------------
#vetor com as condições iniciais MS
#----------------------------------------------------------- 
    y0 = np.array([[xod],[dxod],[yod],[dyod],[zod],[dzod]])
#-----------------------------------------------------------
#vetor com os parâmetros constantes
#-----------------------------------------------------------         
    params = np.array([[m],[L],[g],[k],[Bss],[expK]])
#-----------------------------------------------------------
#Parâmetros para os métodos 
#-----------------------------------------------------------   
    t = 0 #inicio do tempo  t = 0
    h = hEdo #passo do método rungeKutta42 inicial
    N = 10000 #número máximo de iterações
    y = np.zeros((6,1))
    #primeiro método
    sh = h #tamanho do passo para o método rungeKutta42 atualizando durante a execução do método
    ind = 0 #contador
#-----------------------------------------------------------
#vetores auxiliares para guardar a trajetória
#-----------------------------------------------------------    
    px = [y0[0,0]]
    py = [y0[2,0]]
    pz = [y0[4,0]]
#-----------------------------------------------------------
#inicio do método primeiro MS para TD
#----------------------------------------------------------- 
    for x in np.arange(0,N*h,h):
#-----------------------------------------------------------
#vetor de parâmetros
#-----------------------------------------------------------  
        var = np.array([[t],[h],[1]])
#-----------------------------------------------------------
#método numérico para solucionar as equações diferenciais
#passo a passo
#----------------------------------------------------------- 
        y = rungeKutta42(var,y0,params)
#-----------------------------------------------------------
#atualizando a condição inicial
#----------------------------------------------------------- 
        y0  = y
#-----------------------------------------------------------
#atualizando o instante t
#----------------------------------------------------------- 
        t = t+sh
#-----------------------------------------------------------
#verificando a condição de parada posição Z < que Z de touchdown
#Z de touchdown = L*cos(theta)
#-----------------------------------------------------------  
        if y0[4,0] < L*mt.cos(theta):
         break  
        
#-----------------------------------------------------------
#colocando os valores nos vetores auxiliares
#-----------------------------------------------------------  
        px.append(y0[0,0])
        py.append(y0[2,0])
        pz.append(y0[4,0])
#-----------------------------------------------------------
#atualizando o contador
#-----------------------------------------------------------  
        ind = ind + 1
    
#-----------------------------------------------------------
#atualizando o contador - tratando o valor
#-----------------------------------------------------------    
        #if ind > 1:
            #ind = ind -1
    
#-----------------------------------------------------------
#Posição do centro de massa no momento de  Touchdown (TD)
#-----------------------------------------------------------
    pc = np.array([[px[ind]],[py[ind]],[pz[ind]]])#centro de massa
    px= np.asarray(px)
    py= np.asarray(py)
    pz= np.asarray(pz)
#-----------------------------------------------------------
#posição do pé de balaço quando toca o chão
#-----------------------------------------------------------    
    pfb = pc + L*np.array([mt.sin(theta)*mt.cos(phi),mt.sin(theta)*mt.sin(phi),-mt.cos(theta)])
#-----------------------------------------------------------
#tempo em que acontece a codição de touchdown
#----------------------------------------------------------- 
    TD = t #tempo de TD
#-----------------------------------------------------------
#parâmetros constante para o segundo método
#-----------------------------------------------------------
    params = np.array([[m],[L],[g],[k],[Bss],[t],[pfb[0,0]],[pfb[1,0]],[pfb[2,0]],[expK]])
#-----------------------------------------------------------
#iniciando o segundo contador
#-----------------------------------------------------------
    ind2 = 0
    sh = h #tamanho do passo para o método rungeKutta42 atualizando durante a execução do método
#-----------------------------------------------------------
#vetores auxiliares para guardar a trajetória
#----------------------------------------------------------- 
    px2 = [y0[0,0]]
    py2 = [y0[2,0]]
    pz2 = [y0[4,0]]
#-----------------------------------------------------------
#inicio do método 2 - TD para LH
#----------------------------------------------------------- 
    for x in np.arange(0,N*h,h):
#-----------------------------------------------------------
#vetor de parâmetros
#-----------------------------------------------------------    
        var = np.array([[t],[h],[0]])
#-----------------------------------------------------------
#método numérico para solucionar as equações diferenciais
#passo a passo
#-----------------------------------------------------------
        y = rungeKutta42(var,y0,params)
#-----------------------------------------------------------
#atualizando nova condição inicial
#-----------------------------------------------------------     
        y0  = y
#-----------------------------------------------------------
#atualizando o instante t
#-----------------------------------------------------------
        t = t+sh    
#-----------------------------------------------------------
#verificando a condição de parada posição dZ > 0
#-----------------------------------------------------------
        if y0[5,0] > 0:
            break
#-----------------------------------------------------------
#atualizando o contador
#-----------------------------------------------------------
        ind2 = ind2+1       

#-----------------------------------------------------------
#atualizando os vetores auxiliares da trajetória
#-----------------------------------------------------------
        px2.append(y0[0,0])
        py2.append(y0[2,0])
        pz2.append(y0[4,0]) 
#-----------------------------------------------------------
#atualizando o contador 
#-----------------------------------------------------------
    #if ind2 > 1:
        #ind2 = ind2 -1
    
#-----------------------------------------------------------
#atualizando a posição do centro de massa
#-----------------------------------------------------------
    pc = np.array([px2[ind2],py2[ind2],pz2[ind2]]) #centro de massa
    px2= np.asarray(px2)
    py2= np.asarray(py2)
    pz2= np.asarray(pz2)
#-----------------------------------------------------------    # trajetoria 
# a trajetoria é simetrica ao ponto de middle suport
# desta forma fazemos um espelho da função 
#deslocado ela para a origem
#-----------------------------------------------------------    
    #plot3(px(1,1:ind),py(1,1:ind),pz(1,1:ind),'b')
    #plot3(px2(1,1:ind2),py2(1,1:ind2),pz2(1,1:ind2),'r')
    plot3 = plt.figure(px[0:ind],py[0:ind],pz[0:ind],'b')
    plot3 = plt.figure(px2[0:ind2],py2[0:ind2],pz2[0:ind2],'r')
   
#-----------------------------------------------------------
#espelho da função
#-----------------------------------------------------------  
    offsetx = px2[ind2]
    offsety = py2[ind2]
    plot3 = plt.figure(-px2 + 2*offsetx,-py2+2*offsety,pz2,'r')
    offsetx = -px2[ind2] + 2*offsetx
    offsety = -py2[ind2] + 2*offsety
    plot3 = plt.figure(-px[range(ind,-1,-1)] + 2*offsetx,-py[range(ind,-1,-1)]+2*offsety,pz[range(ind,-1,-1)],'b')
    
#-----------------------------------------------------------
#projeção
#-----------------------------------------------------------   
    plot3 = plt.figure(px[0:ind],py[0,0:ind],0*pz[0:ind],'g')
    plot3 = plt.figure(px2[0,0:ind2],py2[0,0:ind2],0*pz2[0:ind2],'m')
#-----------------------------------------------------------
#espelho da função
#-----------------------------------------------------------    
    offsetx = px2[0,ind2]
    offsety = py2[0,ind2]
    plot3 = plt.figure(-px2([range(ind2,-1,-1)]) + 2*offsetx,-py2[range(ind2,-1,-1)]+2*offsety,0*pz2[range(ind2,-1,-1)],'m')
    offsetx = -px2[0,ind2] + 2*offsetx
    offsety = -py2[0,ind2] + 2*offsety
    plot3 = plt.figure(-px[range(ind,-1,-1)] + 2*offsetx,-py[range(ind,-1,-1)]+2*offsety,0*pz[range(ind,-1,-1)],'g')
    
#-----------------------------------------------------------
#plotar posição dos pés
#----------------------------------------------------------- 
    plot3 = plt.figure([pfa[0,0], pfb[0,0]],[pfa[1,0], pfb[1,0]],[0, 0],'--k')
#-----------------------------------------------------------
#projeção centro de massa e projeção centro de massa
#-----------------------------------------------------------     
    plot3 = plt.figure(pc[0,0],pc[1,0],pc[2,0],'o')

    plot3 = plt.figure([pc[0,0], pc[0,0]],[pc[1,0], pc[1,0]],[pc[2,0], 0],':')
    
    plot3 = plt.figure(pc[0,0],pc[1,0],0,'x')
#-----------------------------------------------------------
#configuração do grafico
#-----------------------------------------------------------     
    plot3 = plt.figure(figsize=(60,35))
    ax = plot3.add_subplot(111,projection='3d')
    plt.grid(True)
    ax.set_xlim([-1,1])
    ax.set_ylim([-0.5,0.5])
    ax.set_zlim([-1,-0.2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.show()


    
