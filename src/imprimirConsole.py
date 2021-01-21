#-----------------------------------------------------------
#Imprime no console os valores - usado para verificar o código
#Parâmetros 
#iteracao - número da iteração que deseja imprimir no console
#vec      - vetor com os valores a serem imprimidos
#theta - U[1,1]
#phi   - U(2,1)
#K     - U(3,1)
#BSS   - U(4,1)
#fo    - U(5,1)
#----------------------------------------------------------
def imprimirConsole(iteracao,vec):
#-----------------------------------------------------------
#separar os valores do vetor
#-----------------------------------------------------------     
     U = vec[0]
     theta = U[0,0]
     phi   = U[1,0]
     K     = U[2,0]
     BSS   = U[3,0]
     expK  = U[4,0]
     fo    = vec[1]
#-----------------------------------------------------------
#imprimir no console
#-----------------------------------------------------------     
     print('------------------------------------------------------------')
     
     print('iteracao : ' + str(iteracao))
     
     print('phi = ' + str(phi))
     
     print('theta = ' + str(theta))
     
     print('k = '+ str(K*expK))
     
     print('Bss = ' + str(BSS))
     
     print('FO : ' + str(fo))
     