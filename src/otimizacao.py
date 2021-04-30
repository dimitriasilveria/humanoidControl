import numpy as np 
from NAG import NAG
from adagrad import adagrad
from SGD import SGD
from SGDmomento import SGDMomento
from funcaoObjetivo import funcaoObjetivo
from trajetoria import trajetoria
from imprimirConsole import imprimirConsole
from setLimites import setLimites
from plotarTrajetoria import plotarTrajetoria
#-------------------------------------
# ----------------------
#método para calcular os melhores parâmetros para a variável de controle U
#minimizando uma função objetivo.
#Otimizações implementadas:
#metodo : 0 estocástico  gradiente descendente SGD
#metodo : 1 SGD com momento
#metodo : 2 Nesterov accelerated gradient
#metodo : 3 Adagrad
#o método escolhido será aquele que apresentar melhor convergência
#Parâmetros:
#tipo: 1 para executar a otimização e plotar a trajetória dos valores obtidos pela otimização
#tipo:0 apenas plotar a trajetória dos valores de U já estipulados
#U - variável de controle inicial
#X - condição inicial constante
#Retorno:
#mostrado no console o melhor resultado e mostrado graficamente a
#trajetória obtida
#no caso do robô, essa trajetória será mostrada no próprio
#-----------------------------------------------------------
import matplotlib.pyplot as plt
from globalVariables import GlobalVariables

def otimizacao(U,X,tipo,metodo):
    if tipo == 1:
        #-----------------------------------------------------------
        #variáveis globais
        #-----------------------------------------------------------
        glob = GlobalVariables()
        maxNGrad = glob.getMaxNGrad()
        #ganhoAlpha = glob.getGanhoAlpha
        #gamma = glob.getGamma
        #global maxNGrad, ganhoAlpha, gamma 
        #-----------------------------------------------------------
        #iniciar variáveis de controle inicial
        #-----------------------------------------------------------
        u1 = U[0,0]
        u2 = U[1,0]
        u3 = U[2,0]
        u4 = U[3,0]
        u5 = U[4,0]
        #-----------------------------------------------------------
        #inicio do método
        #----------------------------------------------------------
        fo = 1 #condição de parada
        #-----------------------------------------------------------
        #melhores valores
        #----------------------------------------------------------
        fm = fo
        UM = U
        #-----------------------------------------------------------
        #vetores axiliares para os métodos de otimização
        #----------------------------------------------------------
        vt = np.zeros((4,1))  #usado como auxiliar NAG
        Grad = np.zeros((4,1)) #usado como auxiliar adagrad

        [pa,pb,pc,M,ponto] = trajetoria(U,X)     
        fo = funcaoObjetivo(pa,pb,pc)
        if fo < 1*10**(-10): 
            print("Valores já otimizados")
            return
        
        for j in range(1, maxNGrad,1):
            
            #-----------------------------------------------------------
            # gradiente descendente estocástico SGD
            #---------------------------------------------------------- 
            if metodo == 0:      
                U = SGD(U,X)
            
            #-----------------------------------------------------------
            #SGD com momento
            #---------------------------------------------------------- 
            if metodo == 1:      
                [U,vt] = SGDMomento(U,X,vt)
            
            #-----------------------------------------------------------
            #Nesterov accelerated gradient
            #---------------------------------------------------------- 
            if metodo == 2:      
                [U,vt] = NAG(U,X,vt)
            
            #-----------------------------------------------------------
            #Adagrad
            #---------------------------------------------------------- 
            if metodo == 3:      
                [U,Grad] = adagrad(U,X,Grad)
            
            
            #-----------------------------------------------------------
            #Setar os limites inferiores e superiores em U
            #----------------------------------------------------------  
            U0 = np.zeros((5,1))
            U0 = setLimites(U)
            u1 = U0[0,0]
            u2 = U0[1,0]
            u3 = U0[2,0]
            u4 = U0[3,0]
            u5 = U0[4,0]
            #-----------------------------------------------------------
            #atualizar o vetor U  (variáveis de controle)
            #----------------------------------------------------------
            U  = np.array([[u1],[u2],[u3],[u4],[u5]])
            #-----------------------------------------------------------
            #Cálculo do valor da função objetivo
            #a função de otimização é usada para calcular os valores de U, 
            #que serão inseridos na função de calcular a trajetória
            #----------------------------------------------------------              
            [pa,pb,pc,M,ponto] = trajetoria(U,X)     
            fo = funcaoObjetivo(pa,pb,pc)
            #-----------------------------------------------------------
            #verificar melhor resultado
            #fm = 1, inicialmente
            #cada vez que fo < fm, fm armazena o valor de fo
            #fo sempre é comparado com seu valor anterior, desde que 
            #esteja convergindo
            #valores de fo maiores que o anterior, serão ignorados
            #----------------------------------------------------------      
            if fo<fm:
                fm = fo
                UM = U
                
            #-----------------------------------------------------------
            #verificar condição de parada
            #a condição de parada ocorre quando a projeção do CoM no plano xy
            #praticamente coincide com o ponto médio das duas pernas, no mesmo plano
            #isso deve acontecer na fase LH, da caminhada
            #----------------------------------------------------------  
            if fo < 1*10**(-10): 
                break
            
            #-----------------------------------------------------------
            #imprimir resultado no console
            #----------------------------------------------------------     
            imprimirConsole(j,[U,fo])             
        
        #-----------------------------------------------------------
        #imprimir resultado final no console
        #---------------------------------------------------------- 
        print('************************************************************')
        print('Melhor Solução: ')
        imprimirConsole(0,[UM,fm])
        print('************************************************************')
        #-----------------------------------------------------------
        #mostrar a trajetória
        #----------------------------------------------------------
        plotarTrajetoria(UM,X)
    else:

        #-----------------------------------------------------------
        #mostrar a trajetoria
        #----------------------------------------------------------
        plotarTrajetoria(U,X)