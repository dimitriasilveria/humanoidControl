from trajetoria import trajetoria
from trajetoriaPes import trajetoriaPes
from fase3 import fase3
from fase1 import fase1
from fase2 import fase2
from globalVariables import GlobalVariables
from trajetoriaPesInicio import trajetoriaPesInicio
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
def caminhada2(U0,X0,tam,vecGanho1,vecGanho2):
    #-----------------------------------------------------------
    #Obter todas as trajeotrias do CoM
    #-----------------------------------------------------------
    import numpy as np
    [PA,PB,PC,trajCoM1,indContadoPe] = trajetoria(U0,X0) #trajetória para o CoM
    #trajetoria 2
    trajCoM = trajCoM1
    #print(np.size(trajCoM1))
    ind = np.size(trajCoM,0) #pegar a última posição do vetor de pontos
    #a partir daqui vai dentro do loop

    #até aqui vai dentro do loop
    #-----------------------------------------------------------
    #Trajetória dos pés
    #-----------------------------------------------------------
    passoComprimento = PB[0,0] #tamanho do passo
    passoLargura     = PB[1,0] #Largura do passo
    passoAltura      = 0.2    #altura de cada passo

    #trajetoria pé B inicial
    tamTrajPeB1 = indContadoPe
    #trajPB1 = trajetoriaPes(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,1,tamTrajPeB1)
    trajPB1 = trajetoriaPesInicio(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,tamTrajPeB1)
    
    
    passoComprimento2 = PB[0,0] #tamanho do passo
    passoLargura2     = 0 #Largura do passo
    passoAltura2      = 0.2    #altura de cada passo
    
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
    h = glob.getH()
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

    un = U0
        
    #primeira parte da caminhdada
    #print('primeiro passinho')
    passos = 1
    [ha,ha2,theta,tempo1] = fase1(trajCoM1,indContadoPe,trajPB1,theta,vecGanho1)
    
    if passos >=tam:
        return
    
    tempo = tempo+tempo1
    i = 0   
    while 1:
        glob = GlobalVariables()
        dt = glob.getHEDO()
        n = ind
        dx = (trajCoM[n-1,0] - trajCoM[n-2,0])/dt
        dy = (trajCoM[n-1,1] - trajCoM[n-2,1])/dt
        #dz = (trajCoM3_1[n-1,2] - trajCoM3_1[n-2,2])/dt
        dz = 0
        #print(X0[0:3,:])
        x = X0[0:3,:] + np.array([[0.43605039],[0.05947525],[0.0]])
        #print(x)
        v = np.array([[dx],[dy],[dz]])
        #print(v)
        xn = np.concatenate((x,v),axis = 0)
        un = LRQ3D(U0,X0,passos,tempo,xn,i) #aqui deve ser rodado em paralelo
        #trajCoM2 = np.zeros((ind,3))
        trajCoM2_1 = np.zeros((ind,3))
        trajCoM2_2 = np.zeros((ind,3))

        #calculo da trajetória do CoM na fase2:

        offsetx = trajCoM[ind-1,0]#cálcular o offset em x
        offsety = trajCoM[ind-1,1]#calcular o offset em y
        trajCoM2_1[:,0] = -trajCoM[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
        trajCoM2_1[:,1] = -trajCoM[range(ind-1,-1,-1),1] + 2*offsety #calcular a trajetória simétrica para y
        trajCoM2_1[:,2] =  trajCoM[range(ind-1,-1,-1),2] #em z não muda
        #trajCoM2_2 = np.zeros((ind,3)) #o tamanho da trajetória será sempre o mesmo???????????????
        #Aqui será feito o espelhamento da trajetória
        offsetx = trajCoM2_1[ind-1,0] #cálcular o offset em x
        offsety = trajCoM2_1[ind-1,1] #calcular o offset em y
        trajCoM2_2[:,0] = -trajCoM2_1[range(ind-1,-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
        trajCoM2_2[:,1] = trajCoM2_1[range(ind-1,-1,-1),1] #calcular a trajetória simétrica para y
        trajCoM2_2[:,2] =  trajCoM2_1[range(ind-1,-1,-1),2] #em z não muda

        trajCoM2 = np.concatenate((trajCoM2_1, trajCoM2_2),axis=0)
    
        passoTrajCoM = trajCoM2[np.size(trajCoM2,0)-1,0] - trajCoM2[0,0]
    
        
        trajCoM = trajCoM2
        #for j in range(np.size(trajCoM,0)):
        trajCoM[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
        trajP = trajPA
        #k = np.size(trajP,0)
        #for j in range(k):
        trajP[:,0] = trajP[:,0]+ i*2*passoComprimento

        passos = passos + 1
        [ha,ha2,theta,tempo2] = fase2(ha,ha2,trajCoM,np.size(trajPA,0),trajP,theta,tempo,vecGanho2)
        if passos >=tam: #tam é a quantidade de passos da trajetória desejada
            return
        #aqui começa a fase3#########################################################
        tempo=tempo+tempo2
 
        #É necessário recalcular a trajetória do CoM com o novo vetor u
        
        #ind = np.size(trajCoM2_1,0) #pegar a última posição do vetor de pontos
        trajCoM3_1 = np.zeros((ind,3))
        #isso está correto???????????????????????????
        offsetx = trajCoM2_1[(ind-1),0] #cálcular o offset em x
        offsety = trajCoM2_1[(ind-1),1] #calcular o offset em y
        #clr trajCoM3
        trajCoM3_1[:,0] = -trajCoM2_1[range((ind-1),-1,-1),0] + 2*offsetx #calcular a trajetória simétrica para x
        trajCoM3_1[:,1] = -trajCoM2_1[range((ind-1),-1,-1),1] + 2*offsety #calcular a trajetória simétrica para y
        trajCoM3_1[:,2] =  trajCoM2_1[range((ind-1),-1,-1),2] #em z não muda
        #calculando as velocidades
        #glob = GlobalVariables()
        #dt = glob.getHEDO()
        #n = np.size(trajCoM3_1,0)
        #dx = (trajCoM3_1[n-1,0] - trajCoM3_1[n-2,0])/dt
        #dy = (trajCoM3_1[n-1,1] - trajCoM3_1[n-2,1])/dt
        #dz = (trajCoM3_1[n-1,2] - trajCoM3_1[n-2,2])/dt
        #dz = 0
        #v = np.array([dx,dy,dz])
        #print(v)
        #print(U0)
        #print(X0)
        #print(trajCoM[n-1,:].reshape((3,1)) - X0[0:3,0])
        #xn = np.concatenate((trajCoM[n-1,:],v),0).reshape((6,1))
        #dx = xn - X0
        #print(dx)
        #un = LRQ3D(U0,X0,passos,tempo,xn,i)
        u0 = np.array([un[0,0],un[1,0],U0[2,0],un[2,0],U0[4,0]]).reshape((5,1))
        [PA,PB,PC,trajCoM3_2,indContadoPe] = trajetoria(u0,xn)
        X0 = xn
        U0 = u0
        #print(U0)
        ind = np.size(trajCoM3_1,0) + np.size(trajCoM3_2,0)
        trajCoM3 = np.zeros((ind,3)) 
        trajCoM3 = np.concatenate((trajCoM3_1,trajCoM3_2),axis = 0)
        trajCoM = trajCoM3
        #for j in range(np.size(trajCoM,0)):
        trajCoM[:,0] = trajCoM[:,0] + i*2*passoTrajCoM
        trajP = trajPB
        #k = np.size(trajP,0)
        #for j in range(k):
        trajP[:,0] = trajP[:,0]+ i*2*passoComprimento
        #A cada 2 passos ou quando a velocidade muda, é necessário chamar o 
        #LQR do modelo 3D
        
        
        #quem serão xn e pc??????????????????????????????????????????????????
        
        #CoM = trajCoM
        ind = np.size(trajCoM,0)  
        passos = passos + 1
        [ha,ha2,theta,tempo3] = fase3(ha,ha2,trajCoM,np.size(trajPB,0),trajP,theta,tempo,vecGanho1)
        
        if passos >=tam:
            return
        
        tempo=tempo+tempo3
        i = i+1

















    


