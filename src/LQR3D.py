import numpy as np 
from jacobianoF import jacobianoF
from globalVariables import GlobalVariables
from trajetoria import trajetoria

#xn e un são os vetores atuais (reais)
#Xn e Un são os desejados
#U0 e X0 são as condições iniciais
#X = [x,y,z,dx,dy]
#U = [phi, theta, k] (model1)
#U = [phi,theta,Bss] (model2)
def LRQ3D(U0,X0,passo,t,xn,i):
    #deltaU = np.zeros((5,1))
    glob = GlobalVariables()
    #L = glob.getL

    #theta = un[1,0]
    #phi = un[0,0]
    #X0 = X0[0:5,:]
    #U0 = U0[0:3,:]

    A = np.diag([1,-1,1,1,-1])
    B = np.diag([-1,1,1])
    #Xn = (A^n)*X0
    Xn = np.dot(np.linalg.matrix_power(A,passo),X0[0:5,:])
    Un = np.dot(np.linalg.matrix_power(B,passo),U0[0:3,:])
    
    #pfb = pc + L*np.array([[np.sin(theta)*np.cos(phi)],[np.sin(theta)*np.sin(phi)],[-np.cos(theta)]])
    #pfb = np.array([[2.5], [6.80], [5.8]])
    #f = np.dot(A,Xn)
    
    [Jx, Ju] = jacobianos(A,xn,X0,U0,i)
    xn = xn[0:5,:]
    #Jx = A*(df/dx) avaliado em (X0,U0)
    #Ju = A*(df/du) avaliado em (X0,U0)

    #Aqui começa o LRQ:
    #definindo os ganhos:
    Q = 0.01*np.eye(5)
    R = 0.2*np.eye(3)
    #S = 0.08*np.eye(5)

    #resolvendo a equação de Ricatti para encontrar P:
    #P = Jx.T*(P-P*Ju*np.inv(B.T*P*Ju+R)*Ju.T*P)*Jx+Q
    #P = Q #setando Pf=S
    #MP = np.zeros((5,5,N))
    #for j in range(5):
    #    for k in range(5):
    #        MP2[j,k,T-1] = P[j,k]
    #def Pfun(P):
    #    JuTransP = np.dot(Ju.T,P) #Ju.T*P 
    #    JutransPJu = np.dot(JuTransP,Ju) + R #Ju.T*P*Ju+R
    #    PJu = np.dot(P,Ju) #P*Ju
    #    Inv = np.linalg.inv(JutransPJu) #(Ju.T*P*Ju+R)^-1
    #    prod = np.dot(PJu,Inv) #P*Ju*np.inv(B.T*P*Ju+R) 
    #    prod1 = np.dot(prod,JuTransP) #P*Ju*np.inv(B.T*P*Ju+R)*Ju.T*P
    #    Sub = P-prod1 #P-P*Ju*np.inv(B.T*P*Ju+R
    #    prod3 = np.dot(Jx.T, Sub) #Jx.T*(P-P*Ju*np.inv(B.T*P*Ju+R)
        #P = np.dot(prod3,JuTransP) + Q
    #    return P - np.dot(prod3,JuTransP) - Q  #retorna a função que 
        #será usada para encontrar P com o método de raízes, do python

    #from scipy.optimize import fsolve
    #P = fsolve(Pfun,1)
    #calculando o ganho K, do controlador
    from scipy import linalg as lg 
    P = lg.solve_discrete_are(Jx, Ju, Q, R)
    K = -np.linalg.inv((Ju.T)@P@Ju+R)@(Ju.T)@P@Jx 
    #n realmente é passo ????????????????????????????????????????????????????/
    #calculando o vetor de parâmetros controlado
    un = Un + np.linalg.matrix_power(B,passo)@K@(np.linalg.matrix_power(A,passo)@(xn - Xn))
   
    return un

def TrajetoriaVariandoX(X0,U0,i,indice):
    #funçõa para calcular a trajetória variando X0 e U0, para calcular os jacobianos
    glob = GlobalVariables()
    h = glob.getH()
    X0[indice,0] = X0[indice,0] + h
   
    #trajetórias na fase 1
    [PAx,PBx,PCx,trajCoMX1,indContadoPex] = trajetoria(U0,X0)
    
    #trajetórias na fase 2:
    indX = np.size(trajCoMX1,0)
    
    trajCoMX2_1 = np.zeros((indX,3))
    trajCoMX2_2 = np.zeros((indX,3))

    #primeira metade################################################################
    offsetxX = trajCoMX1[indX-1,0]#cálcular o offset em x
    offsetyX = trajCoMX1[indX-1,1]#calcular o offset em y
    trajCoMX2_1[:,0] = -trajCoMX1[range(indX-1,-1,-1),0] + 2*offsetxX #calcular a trajetória simétrica para x
    trajCoMX2_1[:,1] = -trajCoMX1[range(indX-1,-1,-1),1] + 2*offsetyX #calcular a trajetória simétrica para y
    trajCoMX2_1[:,2] =  trajCoMX1[range(indX-1,-1,-1),2] #em z não muda

    #segunda metade#################################################################3
    offsetxX = trajCoMX2_1[indX-1,0] #cálcular o offset em x
    offsetyX = trajCoMX2_1[indX-1,1] #calcular o offset em y
    trajCoMX2_2[:,0] = -trajCoMX2_1[range(indX-1,-1,-1),0] + 2*offsetxX #calcular a trajetória simétrica para x
    trajCoMX2_2[:,1] = trajCoMX2_1[range(indX-1,-1,-1),1] #calcular a trajetória simétrica para y
    trajCoMX2_2[:,2] =  trajCoMX2_1[range(indX-1,-1,-1),2] #em z não muda   
    
    #concatenando as duas:
    trajCoM2X = np.concatenate((trajCoMX2_1, trajCoMX2_2),axis=0)
        
    passoTrajCoMX = trajCoM2X[np.size(trajCoM2X,0)-1,0] - trajCoM2X[0,0]
    
    trajCoM2X[:,0] = trajCoM2X[:,0] + i*2*passoTrajCoMX
    
    #Trajetoria na fase3:
    trajCoMX3_1 = np.zeros((indX,3))
    
    #isso está correto???????????????????????????
    offsetxX = trajCoMX2_1[(indX-1),0] #cálcular o offset em x
    offsetyX = trajCoMX2_1[(indX-1),1] #calcular o offset em y
    #clr trajCoM3
    trajCoMX3_1[:,0] = -trajCoMX2_1[range((indX-1),-1,-1),0] + 2*offsetxX #calcular a trajetória simétrica para x
    trajCoMX3_1[:,1] = -trajCoMX2_1[range((indX-1),-1,-1),1] + 2*offsetyX #calcular a trajetória simétrica para y
    trajCoMX3_1[:,2] =  trajCoMX2_1[range((indX-1),-1,-1),2] #em z não muda

    trajCoMX3_1[:,0] = trajCoMX3_1[:,0] + i*2*passoTrajCoMX
    #Calculando as derivadas
    dt = glob.getHEDO()
    #n = np.size(trajCoMX3_1,0)
    dx = (trajCoMX3_1[indX-1,0] - trajCoMX3_1[indX-2,0])/dt
    dy = (trajCoMX3_1[indX-1,1] - trajCoMX3_1[indX-2,1])/dt
    #dz = (trajCoMX3_1[indX-1,2] - trajCoMX3_1[indX-2,2])/dt
    vx = np.array([dx,dy])
    xnX = np.concatenate((trajCoMX3_1[indX-1,:],vx),0).reshape((5,1))

    return xnX

#U0[0,0] = theta
#U0[1,0] = phi
#U0[2,0] = k 
#U0[3,0] = Bss
#U0[4,0] = expK

def TrajetoriaVariandoU(X0,U0,i,indice):
    #função para calcular a trajetória variando U0, para calcular os jacobianos
    glob = GlobalVariables()
    h = glob.getH()
    dt = glob.getHEDO()
    
    U0[indice,0] = U0[indice,0] + h
    #trajetoria na fase 1
    [PAu,PBu,PCu,trajCoMU1,indContadoPeu] = trajetoria(U0,X0)

    #trajetoria na fase 2
    indU = np.size(trajCoMU1,0)
    trajCoMU2_1 = np.zeros((indU,3))
    trajCoMU2_2 = np.zeros((indU,3))
    #primeira metade #######################################
    offsetxU = trajCoMU1[indU-1,0]#cálcular o offset em x
    offsetyU = trajCoMU1[indU-1,1]#calcular o offset em y
    trajCoMU2_1[:,0] = -trajCoMU1[range(indU-1,-1,-1),0] + 2*offsetxU #calcular a trajetória simétrica para x
    trajCoMU2_1[:,1] = -trajCoMU1[range(indU-1,-1,-1),1] + 2*offsetyU #calcular a trajetória simétrica para y
    trajCoMU2_1[:,2] =  trajCoMU1[range(indU-1,-1,-1),2] #em z não muda
    #segunda metade ###############################################
    offsetxU = trajCoMU2_1[indU-1,0] #cálcular o offset em x
    offsetyU = trajCoMU2_1[indU-1,1] #calcular o offset em y
    trajCoMU2_2[:,0] = -trajCoMU2_1[range(indU-1,-1,-1),0] + 2*offsetxU #calcular a trajetória simétrica para x
    trajCoMU2_2[:,1] = trajCoMU2_1[range(indU-1,-1,-1),1] #calcular a trajetória simétrica para y
    trajCoMU2_2[:,2] =  trajCoMU2_1[range(indU-1,-1,-1),2] #em z não muda
    #concatenando as duas:
    trajCoM2U = np.concatenate((trajCoMU2_1, trajCoMU2_2),axis=0)

    passoTrajCoMU = trajCoM2U[np.size(trajCoM2U,0)-1,0] - trajCoM2U[0,0]
    trajCoM2U[:,0] = trajCoM2U[:,0] + i*2*passoTrajCoMU
    #trajetoria na fase3
    trajCoMU3_1 = np.zeros((indU,3))

    offsetxU = trajCoMU2_1[(indU-1),0] #cálcular o offset em x
    offsetyU = trajCoMU2_1[(indU-1),1] #calcular o offset em y
    #clr trajCoM3
    trajCoMU3_1[:,0] = -trajCoMU2_1[range((indU-1),-1,-1),0] + 2*offsetxU #calcular a trajetória simétrica para x
    trajCoMU3_1[:,1] = -trajCoMU2_1[range((indU-1),-1,-1),1] + 2*offsetyU #calcular a trajetória simétrica para y
    trajCoMU3_1[:,2] =  trajCoMU2_1[range((indU-1),-1,-1),2] #em z não muda
    
    trajCoMU3_1[:,0] = trajCoMU3_1[:,0] + i*2*passoTrajCoMU
    #CALCULANDO as derivadas
    dx = (trajCoMU3_1[indU-1,0] - trajCoMU3_1[indU-2,0])/dt
    dy = (trajCoMU3_1[indU-1,1] - trajCoMU3_1[indU-2,1])/dt
    #dz = (trajCoMU3_1[indU-1,2] - trajCoMU3_1[indU-2,2])/dt
    vu = np.array([dx,dy])
    xnU = np.concatenate((trajCoMU3_1[indU-1,:],vu),0).reshape((5,1))

    return xnU

def derivadaX(xn,X0,U0,indice,i):
    glob = GlobalVariables()
    h = glob.getH()
    
    xnX = TrajetoriaVariandoX(X0,U0,i,indice)

    dFx = (xnX - xn)/h

    return dFx

def derivadaU(xn,X0,U0,indice,i):
    glob = GlobalVariables()
    h = glob.getH()
    
    xnU = TrajetoriaVariandoU(X0,U0,i,indice)

    dFu = (xnU - xn)/h

    return dFu

def jacobianos(A,xn,X0,U0,i):

    Jx = np.zeros((5,5))
    Ju = np.zeros((5,3))
    #Calculo de Jx:
    J0 = TrajetoriaVariandoX(X0,U0,i,0)
    J1 = TrajetoriaVariandoX(X0,U0,i,1)
    J2 = TrajetoriaVariandoX(X0,U0,i,2)
    J3 = TrajetoriaVariandoX(X0,U0,i,3)
    J4 = TrajetoriaVariandoX(X0,U0,i,4)

    Jx[:,0] = J0[:,0]
    Jx[:,1] = J1[:,0]
    Jx[:,2] = J2[:,0]
    Jx[:,3] = J3[:,0]
    Jx[:,4] = J4[:,0]
    Jx = np.dot(A,Jx)
    #Calculo de Ju:
    J0 = TrajetoriaVariandoU(X0,U0,i,0)
    J1 = TrajetoriaVariandoU(X0,U0,i,1)
    J2 = TrajetoriaVariandoU(X0,U0,i,3) #aqui o índice é 3, ao invés
    #de 2, pois a posição U0[2,0] corresponde ao k e, segundo o autor,
    #variar k pode causar instabilidade no modelo, então, só variamos
    #Bss

    Ju[:,0] = J0[:,0]
    Ju[:,1] = J1[:,0]
    Ju[:,2] = J2[:,0]
    Ju = np.dot(A,Ju)

    return Jx, Ju

    


