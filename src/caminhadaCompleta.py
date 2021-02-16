import numpy as np
from trajetorias import trajetorias

def caminhadaCompleta(U0,X0):
    [trajPB1, trajPB, trajPA, trajCoM1, trajCoM2, trajCoM3, passoTrajCoM, passoComprimento] = trajetorias(U0,X0)

    i = 0
    while 1:

        trajCoM2[:,0] = trajCoM2[:,0] + i*2*passoTrajCoM
        
        trajPA[:,0] = trajPA[:,0]+ i*2*passoComprimento

        trajCoM3[:,0] = trajCoM3[:,0] + i*2*passoTrajCoM

        trajPB[:,0] = trajPB[:,0]+ i*2*passoComprimento
