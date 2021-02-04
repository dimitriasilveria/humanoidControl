#---------------------------------
#Plotar gráficos do controle
#--------------------------------

import matplotlib.pyplot as plt
import numpy as np

def plotGraficosControle(t1,dt,T,Pos,Posd,angle,angled,Mha,Mhd,Mtheta,Pos2,Posd2,angle2,angled2,Mha2,Mhd2,Mtheta2,color1,color2):
    
    trajD = 'k:'
    t = np.array(np.arange((0+t1), (T)*dt+t1, dt))
    #t = t[:-1]
    #print(t)
    #print(Pos)

    fig,axs = plt.subplots(2,2)
    axs[0,0].plot(t,Pos[0,:])
    axs[0,0].plot(t,Posd[0,:],trajD)
    axs[0,0].set_xlabel('tempo (s)')
    axs[0,0].set_ylabel('(m)')
    axs[0,0].set_title('Posição X')
    axs[0,0].grid()
    #plt.show()
    #plot(t(1,:),Pos(1,:),color1)
    #plot(t(1,:),Posd(1,:),trajD)
    


    
    axs[0,1].plot(t,Pos[1,:])
    axs[0,1].plot(t,Posd[1,:],trajD)
    axs[0,1].set_xlabel('tempo (s)')
    axs[0,1].set_ylabel('(m)')
    axs[0,1].set_title('Posição Y')
    axs[0,1].grid()
    #plt.show()
    #plot(t(1,:),Pos(2,:),color1)
    #plot(t(1,:),Posd(2,:),trajD)
   

    axs[1,0].plot(t,Pos[2,:])
    axs[1,0].plot(t,Posd[2,:],trajD)
    axs[1,0].set_xlabel('tempo (s)')
    axs[1,0].set_ylabel('(m)')
    axs[1,0].set_title('Posição Z')
    axs[1,0].grid()
    #plt.show()
    #plot(t(1,:),Pos(3,:),color1)
    #plot(t(1,:),Posd(3,:),trajD)
    
    
    axs[1,1].plot(t,angle[:])
    axs[1,1].plot(t,angled[:],trajD)
    axs[1,1].set_xlabel('tempo (s)')
    axs[1,1].set_ylabel('Angulo (rad)')
    axs[1,1].set_title('angle of rotation')
    axs[1,1].grid()
    #plt.show()
    fig.show()
    #plot(t(1,:),angle(1,:),color1)
    #plot(t(1,:),angled(1,:),trajD)
    
    #plotar o quartenion e sua covergencia
    fig2, ((ax1, ax2,ax3), (ax4, ax5, ax6),(ax7, ax8, ax9)) = plt.subplots(3, 3)

    ax1.plot(t,Mha[0,:])
    ax1.plot(t,Mhd[0,:],trajD)
    ax1.set_xlabel('tempo (s)')
    ax1.set_ylabel('')
    ax1.set_title('q1')
    ax1.grid()
    #plt.show()
    #plot(t(1,:),Mha(1,:),color1)
    #plot(t(1,:),Mhd(1,:),trajD)
    

    ax2.plot(t,Mha[1,:])
    ax2.plot(t,Mhd[1,:],trajD)
    ax2.set_xlabel('tempo (s)')
    ax2.set_ylabel('')
    ax2.set_title('q2')
    ax2.grid()
    #plt.show()
    #plot(t(1,:),Mha(2,:),color1)
    #plot(t(1,:),Mhd(2,:),trajD)
   

    ax3.plot(t,Mha[2,:])
    ax3.plot(t,Mhd[2,:],trajD)
    ax3.set_xlabel('tempo (s)')
    ax3.set_ylabel('')
    ax3.set_title('q3')
    ax3.grid()
    #plt.show()
    #plot(t(1,:),Mha(3,:),color1)
    #plot(t(1,:),Mhd(3,:),trajD)
    

    ax4.plot(t,Mha[3,:])
    ax4.plot(t,Mhd[3,:],trajD)
    ax4.set_xlabel('tempo (s)')
    ax4.set_ylabel('')
    ax4.set_title('q4')
    ax4.grid()
    #plt.show()
    #plot(t(1,:),Mha(4,:),color1)
    #plot(t(1,:),Mhd(4,:),trajD)
    

    ax5.plot(t,Mha[4,:])
    ax5.plot(t,Mhd[4,:],trajD)
    ax5.set_xlabel('tempo (s)')
    ax5.set_ylabel('')
    ax5.set_title('q5')
    ax5.grid()
    #plt.show()
    #plot(t(1,:),Mha(5,:),color1)
    #plot(t(1,:),Mhd(5,:),trajD)
    

    ax6.plot(t,Mha[5,:])
    ax6.plot(t,Mhd[5,:],trajD)
    ax6.set_xlabel('tempo (s)')
    ax6.set_ylabel('')
    ax6.set_title('q6')
    ax6.grid()
    #plt.show()
    #plot(t(1,:),Mha(6,:),color1)
    #plot(t(1,:),Mhd(6,:),trajD)
   

    ax7.plot(t,Mha[6,:])
    ax7.plot(t,Mhd[6,:],trajD)
    ax7.set_xlabel('tempo (s)')
    ax7.set_ylabel('')
    ax7.set_title('q7')
    ax7.grid()
    #plt.show()
    #plot(t(1,:),Mha(7,:),color1)
    #plot(t(1,:),Mhd(7,:),trajD)
    

    ax8.plot(t,Mha[7,:])
    ax8.plot(t,Mhd[7,:],trajD)
    ax8.set_xlabel('tempo (s)')
    ax8.set_ylabel('')
    ax8.set_title('q8')
    ax8.grid()
    #plt.show()
    fig2.show()
    #plot(t(1,:),Mha(8,:),color1)
    #plot(t(1,:),Mhd(8,:),trajD)
    
   
    #figure(3)
    fig3, ((ax1, ax2), (ax3, ax4) , (ax5, ax6)) = plt.subplots(3, 2)
    
    ax1.plot(t,Mtheta[0,:])
    ax1.set_xlabel('tempo (s)')
    ax1.set_ylabel('')
    ax1.set_title('o1')
    ax1.grid()
    #plt.show()
    #plot(t(1,:),Mtheta(1,:),color1)
   

    ax2.plot(t,Mtheta[1,:])
    ax2.set_xlabel('tempo (s)')
    ax2.set_ylabel('')
    ax2.set_title('o2')
    ax2.grid()
    #plt.show()
    #plot(t(1,:),Mtheta(2,:),color1)
    
    ax3.plot(t,Mtheta[2,:])
    ax3.set_xlabel('tempo (s)')
    ax3.set_ylabel('')
    ax3.set_title('o3')
    ax3.grid()
    #plt.show()
    #plot(t(1,:),Mtheta(3,:),color1)
  

    ax4.plot(t,Mtheta[3,:])
    ax4.set_xlabel('tempo (s)')
    ax4.set_ylabel('')
    ax4.set_title('o4')
    ax4.grid()
    #plt.show()
    #plot(t(1,:),Mtheta(4,:),color1)
  

    ax5.plot(t,Mtheta[4,:])
    ax5.set_xlabel('tempo (s)')
    ax5.set_ylabel('')
    ax5.set_title('o5')
    ax5.grid()
    #plt.show()
    #plot(t(1,:),Mtheta(5,:),color1)
    

    ax6.plot(t,Mtheta[5,:])
    ax6.set_xlabel('tempo (s)')
    ax6.set_ylabel('')
    ax6.set_title('o6')
    ax6.grid()
    #plt.show()
    fig3.show()
    #plot(t(1,:),Mtheta(6,:),color1)
 
    
    #figure(4)
    fig4, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    ax1.plot(t,Pos2[0,:])
    ax1.plot(t,Posd2[0,:],trajD)
    ax1.set_xlabel('tempo (s)')
    ax1.set_ylabel('(m)')
    ax1.set_title('Posição X')
    ax1.grid()
    #plt.show()
    #plot(t(1,:),Pos2(1,:),color2)
    #plot(t(1,:),Posd2(1,:),trajD)
   

    ax2.plot(t,Pos2[1,:])
    ax2.plot(t,Posd2[1,:],trajD)
    ax2.set_xlabel('tempo (s)')
    ax2.set_ylabel('(m)')
    ax2.set_title('Posição Y')
    ax2.grid()
    #plt.show()
    #plot(t(1,:),Pos2(2,:),color2)
    #plot(t(1,:),Posd2(2,:),trajD)
    

    ax3.plot(t,Pos2[2,:])
    ax3.plot(t,Posd2[2,:],trajD)
    ax3.set_xlabel('tempo (s)')
    ax3.set_ylabel('(m)')
    ax3.set_title('Posição Z')
    ax3.grid()
    #plt.show()
    #plot(t(1,:),Pos2(3,:),color2)
    #plot(t(1,:),Posd2(3,:),trajD)
   

    ax4.plot(t,angle2[:])
    ax4.plot(t,angled2[:],trajD)
    ax4.set_xlabel('tempo (s)')
    ax4.set_ylabel('angle (rad)')
    ax4.set_title('angle of rotation')
    ax4.grid()
    #plt.show()
    fig4.show()
    #plot(t(1,:),angle2(1,:),color2)
    #plot(t(1,:),angled2(1,:),trajD)
    


    #plotar o quartenion e sua covergencia
    fig5, ((ax1, ax2,ax3), (ax4, ax5, ax6),(ax7, ax8, ax9)) = plt.subplots(3, 3)

    ax1.plot(t,Mha2[0,:])
    ax1.plot(t,Mhd2[0,:],trajD)
    ax1.set_xlabel('tempo (s)')
    ax1.set_ylabel('')
    ax1.set_title('q1')
    ax1.grid()
    #plt.show()
    #plot(t(1,:),Mha2(1,:),color2)
    #plot(t(1,:),Mhd2(1,:),trajD)
    

    ax2.plot(t,Mha2[1,:])
    ax2.plot(t,Mhd2[1,:],trajD)
    ax2.set_xlabel('tempo (s)')
    ax2.set_ylabel('')
    ax2.set_title('q2')
    ax2.grid()
    #plt.show()
    #plot(t(1,:),Mha2(2,:),color2)
    #plot(t(1,:),Mhd2(2,:),trajD)
    

    ax3.plot(t,Mha2[2,:])
    ax3.plot(t,Mhd2[2,:],trajD)
    ax3.set_xlabel('tempo (s)')
    ax3.set_ylabel('')
    ax3.set_title('q3')
    ax3.grid()
    #plt.show()
    #plot(t(1,:),Mha2(3,:),color2)
    #plot(t(1,:),Mhd2(3,:),trajD)
    

    ax4.plot(t,Mha2[3,:])
    ax4.plot(t,Mhd2[3,:],trajD)
    ax4.set_xlabel('tempo (s)')
    ax4.set_ylabel('')
    ax4.set_title('q4')
    ax4.grid()
    #plt.show()
    #plot(t(1,:),Mha2(4,:),color2)
    #plot(t(1,:),Mhd2(4,:),trajD)
    

    ax5.plot(t,Mha2[4,:])
    ax5.plot(t,Mhd2[4,:],trajD)
    ax5.set_xlabel('tempo (s)')
    ax5.set_ylabel('')
    ax5.set_title('q5')
    ax5.grid()
    #plt.show()
    #plot(t(1,:),Mha2(5,:),color2)
    #plot(t(1,:),Mhd2(5,:),trajD)
    

    ax6.plot(t,Mha2[5,:])
    ax6.plot(t,Mhd2[5,:],trajD)
    ax6.set_xlabel('tempo (s)')
    ax6.set_ylabel('')
    ax6.set_title('q6')
    ax6.grid()
    #plt.show()
    #plot(t(1,:),Mha2(6,:),color2)
    #plot(t(1,:),Mhd2(6,:),trajD)
    

    ax7.plot(t,Mha2[6,:])
    ax7.plot(t,Mhd2[6,:],trajD)
    ax7.set_xlabel('tempo (s)')
    ax7.set_ylabel('')
    ax7.set_title('q7')
    ax7.grid()
    #plt.show()
    #plot(t(1,:),Mha2(7,:),color2)
    #plot(t(1,:),Mhd2(7,:),trajD)
    

    ax8.plot(t,Mha2[7,:])
    ax8.plot(t,Mhd2[7,:],trajD)
    ax8.set_xlabel('tempo (s)')
    ax8.set_ylabel('')
    ax8.set_title('q8')
    ax8.grid()
    #plt.show()
    fig5.show()
    #plot(t(1,:),Mha2(8,:),color2)
    #plot(t(1,:),Mhd2(8,:),trajD)
    

    #figure(6)
    fig6, ((ax1, ax2), (ax3, ax4) , (ax5, ax6)) = plt.subplots(3, 2)
    
    ax1.plot(t,Mtheta2[0,:])
    ax1.set_xlabel('tempo (s)')
    ax1.set_ylabel('')
    ax1.set_title('o1')
    ax1.grid()
    #plt.show()
    #plot(t(1,:),Mtheta2(1,:),color2)
    

    ax2.plot(t,Mtheta2[1,:])
    ax2.set_xlabel('tempo (s)')
    ax2.set_ylabel('')
    ax2.set_title('o2')
    ax2.grid()
    #plt.show()
    #plot(t(1,:),Mtheta2(2,:),color2)
    

    ax3.plot(t,Mtheta2[2,:])
    ax3.set_xlabel('tempo (s)')
    ax3.set_ylabel('')
    ax3.set_title('o3')
    ax3.grid()
    #plt.show()
    #plot(t(1,:),Mtheta2(3,:),color2)
    

    ax4.plot(t,Mtheta2[3,:])
    ax4.set_xlabel('tempo (s)')
    ax4.set_ylabel('')
    ax4.set_title('o4')
    ax4.grid()
    #plt.show()
    #plot(t(1,:),Mtheta2(4,:),color2)
    

    ax5.plot(t,Mtheta2[4,:])
    ax5.set_xlabel('tempo (s)')
    ax5.set_ylabel('')
    ax5.set_title('o5')
    ax5.grid()
    #plt.show()
    #plot(t(1,:),Mtheta2(5,:),color2)
    

    ax6.plot(t,Mtheta2[5,:])
    ax6.set_xlabel('tempo (s)')
    ax6.set_ylabel('')
    ax6.set_title('o6')
    ax6.grid()
    #plt.show()
    fig6.show()

    plt.show(block=True)
    #plot(t(1,:),Mtheta2(6,:),color2)
    
