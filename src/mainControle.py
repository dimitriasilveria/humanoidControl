#-----------------------------------------------------------
#Método principal para o controle LQR
#-----------------------------------------------------------
#-----------------------------------------------------------
#iniciar, limpar memória, fechar todas as janelas
#-----------------------------------------------------------
#clc
#close all
#clear all
#format short
#-----------------------------------------------------------
#adicionar as libs necessárias
#-----------------------------------------------------------
#addpath('debug')
#addpath('derivadas')  
#addpath('edoSolve')  
#addpath('graphics')  
#addpath('modelos')  
#addpath('otimizacaoTrajetoria')  
#addpath('trajetoriaCoM')
#addpath('trajetoriaPes')
#addpath('quaternion_library');
#addpath('dual_quaternion_library');
#addpath('kinematic_dualquartenion_library');
#addpath('transformation');
#addpath('dif_Kinematic');
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
import numpy as np
from caminhada import caminhada
from globalVariables import GlobalVariables
from caminhada2 import caminhada2
import time

begin = time.time()
glob = GlobalVariables()
m = glob.getM()
L = glob.getL()
g = glob.getG()
h = glob.getH()
hEdo = glob.getHEDO()
#global  m, L, g,  h, hEdo

U0 = np.zeros((5,1))

#-----------------------------------------------------------
#condição inicial para MS
#-----------------------------------------------------------
xod  = 0.0001 #x inicial
yod  = 0.05 #y inicial
zod  = 0.6 #z inicial 
dxod = 0.7 #velocidade desejada no MS
dyod = 0.00 #condição de balanço
dzod = 0.00 #velocidade em z (igual a zero condição necessaria)

expK   = 10000 #ordem de grandeza da constante massa-mola

#-----------------------------------------------------------
#variável de controle inicial
#modificar os valores obtidos aqui
#-----------------------------------------------------------

#1m/s
phi = 0.3408040779
theta = 0.4052906627
k = 19196.8680322965
Bss = 0.0415640571

#Ganhos LQR (perna direita) e proporcional (perna esquerda)
ganhoS1 = 0
ganhoQ1 = 1
ganhoR1 = 0.000001
ganhoK1 = 1000

#Ganhos LQR (perna esquerda) e proporcional (perna direita)
ganhoS2 = 0
ganhoQ2 = 1
ganhoR2 = 0.000001
ganhoK2 = 1000

#quantidade de passos
quatidadePassos = 3
#-----------------------------------------------------------
#Não modificar aqui
#-----------------------------------------------------------

k = k/expK #tratar o valor 
U0[0,0] = theta
U0[1,0] = phi
U0[2,0] = k 
U0[3,0] = Bss
U0[4,0] = expK

X0 = np.array([[xod],[yod],[zod],[dxod],[dyod],[dzod]]) #condição inicial

vecGanho1 = np.array([ganhoS1, ganhoQ1, ganhoR1, ganhoK1]).reshape((4,1))
vecGanho2 = np.array([ganhoS2, ganhoQ2, ganhoR2, ganhoK2]).reshape((4,1))

caminhada2(U0,X0,quatidadePassos,vecGanho1,vecGanho2)
end = time.time()
print('execution time:', end - begin)