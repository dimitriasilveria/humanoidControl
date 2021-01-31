import numpy as np 
import math
import cmath
from rungekutta42 import rungeKutta42
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#Método para calcular a trajetória do modelo dinâmico
#otimização de um quarto de passo, resolvendo 2 EDOs
#para implementar o modelo dinâmico completo, é necessário resolver mais 4 EDOs
#retornando os pontos dos pés, CoM e a trajetória
#Parâmetros:
#U - variáveis de controle
#X - vetor com as condições iniciais
#Retorno:
#pa - ponto do pé A
#pb - ponto do pé B
#pc - ponto do centro de massa
#M  - trajetória completa vetores x,y e z
#-----------------------------------------------------------
def trajetoria(U,X):
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
    glob = GlobalVariables()
    m    = glob.getM()    #massa
    L    = glob.getL()    #tamanho da perna
    g    = glob.getG()     #gravidade
    h    = glob.getH()   #passo para o calculo das derivadas
    hEdo = glob.getHEDO() #passo para calculo - EDO
#-----------------------------------------------------------
#condições iniciais para MS (referentes ao CM)
#----------------------------------------------------------- 
    xod  = X[0,0]
    yod  = X[1,0]
    zod  = X[2,0]
    dxod = X[3,0]
    dyod = X[4,0]
    dzod = X[5,0]
#-----------------------------------------------------------
#valores para a otimização valores de u
#-----------------------------------------------------------  
    #u = [theta phi k Bss]
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
    t = 0   #inicio do tempo  t = 0
    h = hEdo #passo do método rungeKutta42 inicial
    N = 10000 #número máximo de iterações
    #primeiro metodo
    sh = h #tamanho do passo para o método rungeKutta42 atualizando durante a execução do método
    
    ind = 0 #contador

    #traj = ind + 1 #tamanho do vetor com os pontos da trajetória
#-----------------------------------------------------------
#vetores auxiliares para guardar a trajetória
#-----------------------------------------------------------        
    px = [y0[0,0]]
    py = [y0[2,0]]
    pz = [y0[4,0]]
#y = np.zeros((6,1))
#-----------------------------------------------------------
#inicio do método 1 MS para TD
#----------------------------------------------------------- 
    for i in range(N): #for de 0 até N*h, com passo h
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
        if y0[4,0] < L*(math.cos(theta)):          
            break 
        
#-----------------------------------------------------------
#colocando os valores nos vetores auxiliares
#-----------------------------------------------------------       
        else:    
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
    #if ind > 1:  #não preciso desse if, porque iniciei o contador no 0
       #ind = ind -1 #o Juan havia iniciado em 1
    #end
    ponto = ind
#-----------------------------------------------------------
#Posição do centro de massa no momento de  Touchdown (TD)
#-----------------------------------------------------------        
    pc = np.array([[px[ind]], [py[ind]], [pz[ind]]]).reshape((3,1)) #centro de massa
#-----------------------------------------------------------
#posição do pé de balanço quando toca o chão
#-----------------------------------------------------------    
    pfb = pc + L*np.array([[np.sin(theta)*np.cos(phi)],[np.sin(theta)*np.sin(phi)],[-np.cos(theta)]])
#-----------------------------------------------------------
#tempo em que acontece a codição de touchdown
#-----------------------------------------------------------   
    TD = t#tempo de TD  
#-----------------------------------------------------------
#parametros constante para o segundo método
#-----------------------------------------------------------
    params = np.array([[m],[L],[g],[k],[Bss],[t],[pfb[0,0]],[pfb[1,0]],[pfb[2,0]],[expK]])

#-----------------------------------------------------------
#iniciando o segundo contador
#-----------------------------------------------------------
    ind2 = 0
    #traj2 = ind2 + 1
    sh = h #tamanho do passo para o método rungeKutta42 atualizando durante a execução do método
#-----------------------------------------------------------
#vetores auxiliares para guardar a trajetória
#-----------------------------------------------------------    
    px2 = [y0[0,0]]
    py2 = [y0[2,0]]
    pz2 = [y0[4,0]]
#-----------------------------------------------------------
#inicio do método 2  TD para LH
#-----------------------------------------------------------     
    for x in range(N):

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
        #if v.all():
        if y0[5,0] > 0:
            break
    
#-----------------------------------------------------------
#atualizando os vetores auxiliares da trajetória
#-----------------------------------------------------------
        else:
            px2.append(y0[0,0])
            py2.append(y0[2,0])
            pz2.append(y0[4,0])
#-----------------------------------------------------------
#atualizando o contador
#-----------------------------------------------------------
            ind2 = ind2+1
    
#-----------------------------------------------------------
#atualizando o contador - tratando o valor
#-----------------------------------------------------------
    #if ind2 > 1
     #   ind2 = ind2 -1;
    
#-----------------------------------------------------------
#trajetória do centro de massa CoM M = [x y z]
#-----------------------------------------------------------
# concatenando as listas, para preencher o vetor M
    pxTot = px + px2
    pyTot = py + py2
    pzTot = pz + pz2   
    pxTot = np.asarray(pxTot)
    pxTot = pxTot.reshape(-1,1)
    pyTot = np.asarray(pyTot)
    pyTot = pyTot.reshape(-1,1)
    pzTot = np.asarray(pzTot)
    pzTot = pzTot.reshape(-1,1)
 
    M = np.concatenate((pxTot, pyTot, pzTot),axis = 1)
    M = M.reshape(-1,3)
#-----------------------------------------------------------
#atualizando a posição do centro de massa
#-----------------------------------------------------------
    pc = np.array([[px2[ind2]],[py2[ind2]],[pz2[ind2]]]).reshape((3,1)) #centro de massa
#-----------------------------------------------------------
#atualizando a posição dos pés e do centro de massa 
#para o retorno da função
#-----------------------------------------------------------    
    pa  = np.array([[0],[0],[0]])
    pb  = np.array([[pfb[0,0]],[pfb[1,0]],[0]])
    pc  = np.array([[pc[0,0]],[pc[1,0]],[pc[2,0]]]) 
    #print(np.shape(pc[1,0]))  

    
    return pa, pb, pc, M, ponto
