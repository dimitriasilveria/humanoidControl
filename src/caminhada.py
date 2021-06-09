from trajetoria import trajetoria
from trajetoriaPes import trajetoriaPes
from fase3 import fase3
from fase1 import fase1
from fase2 import fase2
from globalVariables import GlobalVariables
from trajetoriaPesInicio import trajetoriaPesInicio
from trajetoriaPesInicioBackup import trajetoriaPesInicio2
import numpy as np
from LQR3D import LRQ3D
#-----------------------------------------
#Executar a caminhada
#Parâmetros:
#U:Vetor com as variáveis de controle
#X:Vetor com a condição inicial
#vecGanho1:vetor com os ganhos do controlador para a condição da perna
#          direita em contato com chão
#vecGanho2:vetor com os ganhos do controlador para a condição da perna
#          esquerda em contato com o chão
#------------------------------------------
def caminhada(U0,X0,tam,vecGanho1,vecGanho2):
    #-----------------------------------------------------------
    #Obter todas as trajeotrias do CoM
    #-----------------------------------------------------------
    import numpy as np
    [PA,PB,PC,trajCoM1,indContadoPe] = trajetoria(U0,X0) #trajetória para o CoM
    
    #trajetoria 2
    CoM = trajCoM1
    print(np.size(trajCoM1))
    ind = np.size(CoM,0) #pegar a última posição do vetor de pontos
    trajCoM2 = np.zeros((ind,3))
    trajCoM3 = np.zeros((ind,3))
    offsetx = CoM[ind-1,0]#cálcular o offset em x
    offsety = CoM[ind-1,1]#calcular o offset em y
    trajCoM2[:,0] = -CoM[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
    trajCoM2[:,1] = -CoM[range(ind-1,-1,-1),1] + 2*offsety #calcular a trajetória simétrica para y
    trajCoM2[:,2] =  CoM[range(ind-1,-1,-1),2] #em z não muda


    offsetx = trajCoM2[ind-1,0] #cálcular o offset em x
    offsety = trajCoM2[ind-1,1] #calcular o offset em y
    trajCoM3[:,0] = -trajCoM2[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
    trajCoM3[:,1] = trajCoM2[range(ind-1,-1,-1),1] #calcular a trajetória simétrica para y
    trajCoM3[:,2] =  trajCoM2[range(ind-1,-1,-1),2] #em z não muda

    trajCoM2 = np.concatenate((trajCoM2, trajCoM3),axis=0)
  
    passoTrajCoM = trajCoM2[np.size(trajCoM2,0)-1,0] - trajCoM2[0,0]
    
    
    #trajetoria3
    ind = np.size(trajCoM2,0) #pegar a última posição do vetor de pontos
    trajCoM3 = np.zeros((ind,3))
    offsetx = trajCoM2[ind-1,0] #cálcular o offset em x
    offsety = trajCoM2[ind-1,1] #calcular o offset em y
    #clr trajCoM3
    trajCoM3[:,0] = -trajCoM2[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
    trajCoM3[:,1] = -trajCoM2[range(ind-1,-1,-1),1] + 2*offsety #calcular a trajetória simétrica para y
    trajCoM3[:,2] =  trajCoM2[range(ind-1,-1,-1),2] #em z não muda
     
    #-----------------------------------------------------------
    #Trajetória dos pés
    #-----------------------------------------------------------
    passoComprimento = PB[0,0] #tamanho do passo
    passoLargura     = PB[1,0] #Largura do passo
    passoAltura      = 0.02  #altura de cada passo

    #trajetoria pé B inicial
    tamTrajPeB1 = indContadoPe
    #trajPB1 = trajetoriaPes(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,1,tamTrajPeB1)
    trajPB1 = trajetoriaPesInicio(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,tamTrajPeB1)
    
    
    passoComprimento2 = PB[0,0] #tamanho do passo
    passoLargura2     = 0 #Largura do passo
    passoAltura2      = 0.02    #altura de cada passo
    
    #trajetoria pé A
    tamTrajPa= (np.size(trajCoM1,0)+indContadoPe)/2
    trajPA = trajetoriaPes(np.array([[passoComprimento2+passoComprimento2],[passoLargura2],[0]]),passoComprimento2,passoAltura2,0,tamTrajPa)
    #trajPA = np.asarray(trajPA)
    #trajtoria  pé B
    trajPB = trajPA
    #k = np.size(trajPB,0)
    #for i in range(k):
    trajPB[:,0] = trajPB[:,0] + passoComprimento
    trajPB[:,1] = passoLargura
    
    #--------------------------------------------------------------------------------------
    #Modelo do robô 
    #primeiros testes 
    #implementação começa aquiiii
    #--------------------------------------------------------------------------------------
    #parâmetros necessarios
    glob = GlobalVariables()
    #hpi = glob.getHpi()
    #L1 = glob.getL1()
    #L2 = glob.getL2()
    #L3 = glob.getL3()
    #L4 = glob.getL4()
    #L5 = glob.getL5()
    #height = glob.getHeight()
    #MDH = glob.getMDH()
    #global hpi, L1, L2, L3, L4, L5, height, MDH 
    #hpi = np.pi/2
    #N = 6 #Numero de Juntas
    #Comprimento das Juntas
    #parametros hubo
    #L1 = 0.085
    #L2 = 0.0 #L2 = 0.182;#modificando L2 para que L seja 1 depois rodar a simulação de novo
    #L3 = 0.3
    #L4 = 0.3
    #L5 = 0.0663
    #height = L2+L3+L4+L5 #altura inicial total do robô pé para o centro de massa
    #Parametros de D.H. Perna- tabela do artigo
    thetaR = glob.getOr()
    thetaL = glob.getOl()
    theta = np.concatenate((thetaR,thetaL),axis=1) # parametros variaveis
    tempo = 0

           
    #primeira parte da caminhdada
    passos = 1
    [ha,ha2,theta,tempo1,Mtheta, Mtheta2] = fase1(trajCoM1,indContadoPe,trajPB1,theta,vecGanho1)

    if passos >=tam:
        return
    
    tempo = tempo+tempo1
    i = 0   
    while 1:
        
        trajCoM = trajCoM2
        #for j in range(np.size(trajCoM,0)):
        trajCoM[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
        trajP = trajPA
        #k = np.size(trajP,0)
        #for j in range(k):
        trajP[:,0] = trajP[:,0]+ i*2*passoComprimento

        passos = passos + 1
        [ha,ha2,theta,tempo2,Mtheta, Mtheta2] = fase2(ha,ha2,trajCoM,np.size(trajPA,0),trajP,theta,tempo,vecGanho2)
        if passos >=tam: #tam é a quantidade de passos da trajetória desejada
            return
        
        tempo=tempo+tempo2
        trajCoM = trajCoM3
        #for j in range(np.size(trajCoM,0)):
        trajCoM[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
        trajP = trajPB
        #k = np.size(trajP,0)
        #for j in range(k):
        trajP[:,0] = trajP[:,0]+ i*2*passoComprimento
        #A cada 2 passos ou quando a velocidade muda, é necessário chamar o 
        #LQR do modelo 3D
        #un = LRQ3D(U0,X0,passos,tempo,xn,un,PC) 
        #quem serão xn e pc??????????????????????????????????????????????????
        #[PA,PB,PC,trajCoM,indContadoPe] = trajetoria(un,xn)   
        passos = passos + 1
        [ha,ha2,theta,tempo3,Mtheta, Mtheta2] = fase3(ha,ha2,trajCoM,np.size(trajPB,0),trajP,theta,tempo,vecGanho1)
        
        if passos >=tam:
            return
        
        tempo=tempo+tempo3
        i = i+1
    
    
