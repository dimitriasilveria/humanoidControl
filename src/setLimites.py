import numpy as np 
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#Método para verificar  se a variável de controle
#está dentro de determinados limites
#caso não esteja atualiza o valor desta colocando 
#o valor mínimo ou máximo
#Parâmetro:
#U0 - variáveis de controle
#Retorno:
#U - variável de controle corrigida
#----------------------------------------------------------
def setLimites(U0):
#-----------------------------------------------------------
#variáveis globais
#----------------------------------------------------------
    glob = GlobalVariables()
    thetaM = glob.getThetaM()
    phiM = glob.getPhiM()
    KM = glob.getKM()
    BSSM = glob.getBSSM()
    #global thetaM, phiM, KM, BSSM
#-----------------------------------------------------------
#Separar as variaveis de controle
#----------------------------------------------------------
    u1 = U0[0,0]
    u2 = U0[1,0]
    u3 = U0[2,0]
    u4 = U0[3,0]
    u5 = U0[4,0]
#-----------------------------------------------------------
#tratar os dados da variável de controle - theta
#-----------------------------------------------------------          
    if u1 < 0: 
        u1 = 0
         
        
    if u1 > thetaM:
        u1 = thetaM
        
#-----------------------------------------------------------
#tratar os dados da variável de controle - phi
#-----------------------------------------------------------               
    if u2 < 0:
        u2 = 0
        
        
    if u2 > phiM:
        u2 = phiM
        
#-----------------------------------------------------------
#tratar os dados da variável de controle - K
#-----------------------------------------------------------         
    if u3 < 0:
        u3 = 0
                
    if u3 > KM:
        u3 = KM
#-----------------------------------------------------------
#tratar os dados da variável de controle - BSS
#-----------------------------------------------------------         
    if u4 < 0:
        u4 = 0
                
    if u4 > BSSM:
        u4 = BSSM
        
#-----------------------------------------------------------
#retorno da função
#vetor corrigido
#-----------------------------------------------------------           
    U = np.array([[u1],[u2],[u3],[u4],[u5]])

    return U