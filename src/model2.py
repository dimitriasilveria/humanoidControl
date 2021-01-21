import numpy as np 
#-----------------------------------------------------------
#Calcula o valor das derivadas do modelo dinâmico 2
#Parâmetros
#t   - tempo
#Y0  - Vetor com a condição inicial do sistema
#params: parâmetros do sistema
#m   - massa
#L   - comprimento da perna do modelo
#g   - gravidade
#k   - constante da mola
#Bss - coeficiente angular do modelo
#TD  - valor do tempo de touchdown
#bx  - valor de x do pé B
#by  - valor de y do pé B
#bz  - valor de z do pé B
#expK- valor do ganho da constante da mola (ordem de grandeza)
#Retorno:
#dydt - vetor com primeira e segunda derivada do sistema
#-----------------------------------------------------------
def model2(t,Y0,params): 
#-----------------------------------------------------------
#constantes
#-----------------------------------------------------------
    m   = params[0,0]
    L   = params[1,0]
    g   = params[2,0]
    k   = params[3,0]
    Bss = params[4,0]
    TD  = params[5,0]
    bx  = params[6,0]
    by  = params[7,0]
    bz  = params[8,0]
    expK = params[9,0]
#-----------------------------------------------------------
#condição inicial
#-----------------------------------------------------------       
    x  = Y0[0,0]
    dx = Y0[1,0]
    y  = Y0[2,0]
    dy = Y0[3,0]
    z  = Y0[4,0]
    dz = Y0[5,0]
#-----------------------------------------------------------
#norma de L
#-----------------------------------------------------------    
    norma1 = (x*x + y*y + z*z )**0.5
    norma2 = ((x - bx)**2 + (y - by)**2  + (z - bz)**2  )**0.5
#-----------------------------------------------------------
#derivadas de y
#-----------------------------------------------------------        
    f1 = dx
    f2 = (k*expK/m)*( ( L + Bss*TD - norma1)*(x/norma1) + ( L + Bss*TD - norma2)*( (x - bx)/norma2) )
    f3 = dy
    f4 = (k*expK/m)*( ( L + Bss*TD - norma1)*(y/norma1) + ( L + Bss*TD - norma2)*( (y - by)/norma2) )
    f5 = dz
    f6 = (k*expK/m)*( ( L + Bss*TD - norma1)*(z/norma1) + ( L + Bss*TD - norma2)*( (z - bz)/norma2) ) + g
#-----------------------------------------------------------
#sistema de equações
#-----------------------------------------------------------    
    dydt = np.array([[f1],[f2],[f3],[f4],[f5],[f6]])
    return dydt