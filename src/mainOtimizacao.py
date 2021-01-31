#-----------------------------------------------------------
#Método principal para a otimização do modelo
#retornando no console os valores para as variáveis de controle
#Modelo implementado: 
#3D Dual-SLIP
#Otimizações implementadas: 
#metodo : 0 estocástico  gradiente descendente SGD
#metodo : 1 Nesterov accelerated gradient
#metodo : 2 momento
#metodo : 3 Adagrad
#Variável de Controle:
#U = [theta,phi,k,BSS];
#theta - ângulo de abertura frontal da perna
#phi   - ângulo de abertura lateral da perna
#k     - constante da massa mola do modelo
#BSS   - taxa de crescimento linear da perna durante o passo
#-----------------------------------------------------------

#-----------------------------------------------------------
#iniciar, limpar memória, fechar todas as janelas
#-----------------------------------------------------------
#clc
#close all
#clear
#-----------------------------------------------------------
#adicionar as libs necessárias
#-----------------------------------------------------------
from otimizacao import otimizacao  
import numpy as np
from globalVariables import GlobalVariables
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
#global  m, L, g, lstep, pfa, thetaM, phiM, KM, expK, BSSM 
glob = GlobalVariables()
#massa do corpo
m = glob.getM()
#tamanho da perna
L = glob.getL()
#gravidade
g = glob.getG()
#posição do pé de suporte em MS
pfa = glob.getPfa()
#-----------------------------------------------------------
#Numero maximo de iterações para o metodo do gradiente
#-----------------------------------------------------------
#global maxNGrad, ganhoAlpha,  gamma, h, hEdo
maxNGrad   = glob.getMaxNGrad() #número máximo de iterações método
ganhoAlpha = glob.getGanhoAlpha() #ganho do fator de ganho para cada passo
gamma      = glob.getGamma() #ganho para os método gradiente(momento)
h          = glob.getH() #passo para o calculo das derivadas
#calculado (comparado com a função do matlab)
hEdo       = glob.getHEDO() #passo para o calculo das derivadas
#-----------------------------------------------------------
#condição inicial para MS
#-----------------------------------------------------------
#hubo 2 + velocidade maxima 0.4 m/s

xod  = 0.00
yod  = 0.05
zod  = 0.6
dxod = 0.4
dyod = 0.00
dzod = 0.00

# xod  = 0.00 #x inicial (d - desejado)
# yod  = 0.05 #y inicial
# zod  = 0.204#0.244 #z inicial (muito pequeno apens alguns centimetros)
# dxod = 0.4 #velocidade desejada no MS
# dyod = 0.00 #condição de balanço
# dzod = 0.00 #velocidade em z (igual a zero condição necessaria)

#-----------------------------------------------------------
#Tamanho do passo (não sendo usado ainda estudar como incorporar 
#esse dado)
#-----------------------------------------------------------
lstep = 0.5 + 0.1*(dxod - 1)
#-----------------------------------------------------------
#valores máximos das variáveis de controle
#reduzir o espaço de busca
#-----------------------------------------------------------
thetaM = glob.getThetaM()
phiM   = glob.getPhiM()
KM     = glob.getKM()
expK   = glob.getExpK() #ordem de grandeza da constante massa-mola
BSSM   = glob.getBSSM()
#params = [thetaM;phiM;KM;expK;BSSM];                  
#-----------------------------------------------------------
#variavel de controle inicial
#modificar os valores obtidos aqui
#Geralmente na literatura são usados x para variável de estado
#e u para a variável de controle sendo usados essas letras
#em maiusculo para representação
#-----------------------------------------------------------

#phi = 0.5000000000
#theta = 0.3801423352
#k = 19611.4821640244
#Bss = 0.0275951012

phi = 0.5000000000
theta = 0.3801423352
k = 19611.4821640244
Bss = 0.0275951012

# phi = 0.7854
# theta = 0.2423342738
# #theta = 1.2
# k = 818.4462999033558
# Bss = 0.0
U = np.zeros((5,1))
k = k/expK #tratar o valor 
U[0,0] = theta 
U[1,0] = phi 
U[2,0] = k
U[3,0] = Bss
U[4,0] = expK
#-----------------------------------------------------------
#condição inicial constante
#----------------------------------------------------------
#X = np.zeros((6,1))
X = np.array([[xod],[yod],[zod],[dxod],[dyod],[dzod]])
#-----------------------------------------------------------
#Executa o metodo de otimização
#passando o vetor de controle U
#e o vetor da condição inicial X
#tipo: 1 para executar a otimização e plotar a trajetoria dos 
#dos valores obtidos pela otimização
#tipo:0 apenas plotar a trajetoria dos valores de U ja estipulados
#-----------------------------------------------------------
tipo = 1 #recebe 0 ou 1 (alterar para executar a otimização ou não)
#metodo : 0 estocástico  gradiente descendente SGD
#metodo : 1 SGD com momento
#metodo : 2 Nesterov accelerated gradient
#metodo : 3 Adgrad
metodo = 3
otimizacao(U,X,tipo,metodo) 