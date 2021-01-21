
#---------------------------------
#Plotar gráficos do controle
#--------------------------------
import matplotlib.pyplot as plt

def plotTraj1(t,Pos,Posd,angle,angled,Hplot,Hdplot,Mtheta,Pos2,Posd2,angle2,angled2,Hplot2,Hdplot2,Mtheta2):

    #figure(1)
    fig1, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    
    ax1.plot(t,Pos[0,:])
    ax1.plot(t,Posd[0,:])
    ax1.set_xlabel('iteration')
    ax1.set_ylabel('(cm)')
    ax1.set_title('X position')
    ax1.grid()
    #plot(t(1,:),Pos(1,:))
    #plot(t(1,:),Posd(1,:))
    
    ax2.plot(t,Pos[1,:])
    ax2.plot(t,Posd[1,:])
    ax2.set_xlabel('iteration')
    ax2.set_ylabel('(cm)')
    ax2.set_title('Y position')
    ax2.grid()
    #plot(t(1,:),Pos(2,:))
    #plot(t(1,:),Posd(2,:))
    

    ax3.plot(t,Pos[2,:])
    ax3.plot(t,Posd[2,:])
    ax3.set_xlabel('iteration')
    ax3.set_ylabel('(cm)')
    ax3.set_title('Z position')
    ax3.grid()
    #plot(t(1,:),Pos(3,:))
    #plot(t(1,:),Posd(3,:))
   

    ax4.plot(t,angle[0,:])
    ax4.plot(t,angled[0,:])
    ax4.set_xlabel('iteration')
    ax4.set_ylabel('angle (rad)')
    ax4.set_title('angle of rotation')
    ax4.grid()
    plt.show()
    #plot(t(1,:),angle(1,:))
    #plot(t(1,:),angled(1,:))
    
#plotar o quartenion e sua covergencia
    #figure(2)
    fig2, ((ax1, ax2,ax3), (ax4, ax5, ax6),(ax7, ax8, ax9)) = plt.subplots(3, 3)
    
    ax1.plot(t,Hplot[0,:])
    ax1.plot(t,Hdplot[0,:])
    ax1.set_xlabel('iteration')
    ax1.set_ylabel('')
    ax1.set_title('q1')
    ax1.grid()
    #plot(t(1,:),Hplot(1,:))
    #plot(t(1,:),Hdplot(1,:))
    

    ax2.plot(t,Hplot[1,:])
    ax2.plot(t,Hdplot[1,:])
    ax2.set_xlabel('iteration')
    ax2.set_ylabel('')
    ax2.set_title('q2')
    ax2.grid()
    #plot(t(1,:),Hplot(2,:))
    #plot(t(1,:),Hdplot(2,:))
    

    ax3.plot(t,Hplot[2,:])
    ax3.plot(t,Hdplot[2,:])
    ax3.set_xlabel('iteration')
    ax3.set_ylabel('')
    ax3.set_title('q3')
    ax3.grid()
    #plot(t(1,:),Hplot(3,:))
    #plot(t(1,:),Hdplot(3,:))
    

    ax4.plot(t,Hplot[3:])
    ax4.plot(t,Hdplot[3:])
    ax4.set_xlabel('iteration')
    ax4.set_ylabel('')
    ax4.set_title('q4')
    ax4.grid()
    #plot(t(1,:),Hplot(4,:))
    #plot(t(1,:),Hdplot(4,:))
    

    ax5.plot(t,Hplot[4,:])
    ax5.plot(t,Hdplot[4,:])
    ax5.set_xlabel('iteration')
    ax5.set_ylabel('')
    ax5.set_title('q5')
    ax5.grid()
    #plot(t(1,:),Hplot(5,:))
    #plot(t(1,:),Hdplot(5,:))
    

    ax6.plot(t,Hplot[5,:])
    ax6.plot(t,Hdplot[5,:])
    ax6.set_xlabel('iteration')
    ax6.set_ylabel('')
    ax6.set_title('q6')
    ax6.grid()
    #plot(t(1,:),Hplot(6,:))
    #plot(t(1,:),Hdplot(6,:))
    
    ax7.plot(t,Hplot[6,:])
    ax7.plot(t,Hdplot[6,:])
    ax7.set_xlabel('iteration')
    ax7.set_ylabel('')
    ax7.set_title('q7')
    ax7.grid()
    #plot(t(1,:),Hplot(7,:))
    #plot(t(1,:),Hdplot(7,:))
    

    ax8.plot(t,Hplot[7,:])
    ax8.plot(t,Hdplot[7,:])
    ax8.set_xlabel('iteration')
    ax8.set_ylabel('')
    ax8.set_title('q8')
    ax8.grid()
    plt.show()
    #plot(t(1,:),Hplot(8,:))
    #plot(t(1,:),Hdplot(8,:))
    

    #figure(3)
    fig3, ((ax1, ax2), (ax3, ax4) , (ax5, ax6)) = plt.subplots(3, 2)
    
    ax1.plot(t,Mtheta[0,:])
    ax1.set_xlabel('iteration')
    ax1.set_ylabel('')
    ax1.set_title('o1')
    ax1.grid()
    #plot(t(1,:),Mtheta(1,:))
    

    ax2.plot(t,Mtheta[1,:])
    ax2.set_xlabel('iteration')
    ax2.set_ylabel('')
    ax2.set_title('o2')
    ax2.grid()
    #plot(t(1,:),Mtheta(2,:))
    

    ax3.plot(t,Mtheta[2,:])
    ax3.set_xlabel('iteration')
    ax3.set_ylabel('')
    ax3.set_title('o3')
    ax3.grid()
    #plot(t(1,:),Mtheta(3,:))
    

    ax4.plot(t,Mtheta[3,:])
    ax4.set_xlabel('iteration')
    ax4.set_ylabel('')
    ax4.set_title('o4')
    ax4.grid()
    #plot(t(1,:),Mtheta(4,:))
    

    ax5.plot(t,Mtheta[4,:])
    ax5.set_xlabel('iteration')
    ax5.set_ylabel('')
    ax5.set_title('o5')
    ax5.grid()
    #plot(t(1,:),Mtheta(5,:))
    

    ax6.plot(t,Mtheta[5,:])
    ax6.set_xlabel('iteration')
    ax6.set_ylabel('')
    ax6.set_title('o6')
    ax6.grid()
    plt.show()
    #plot(t(1,:),Mtheta(6,:))
    

    #plotar para os pé  
    #figure(4)
    fig4, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    
    ax1.plot(t,Pos2[0,:])
    ax1.plot(t,Posd2[0,:])
    ax1.set_xlabel('iteration')
    ax1.set_ylabel('(cm)')
    ax1.set_title('X position')
    ax1.grid()
    #plot(t(1,:),Pos2(1,:))
    #plot(t(1,:),Posd2(1,:))
    

    ax2.plot(t,Pos2[1,:])
    ax2.plot(t,Posd2[1,:])
    ax2.set_xlabel('iteration')
    ax2.set_ylabel('(cm)')
    ax2.set_title('Y position')
    ax2.grid()
    #plot(t(1,:),Pos2(2,:))
    #plot(t(1,:),Posd2(2,:))
    

    ax3.plot(t,Pos2[2,:])
    ax3.plot(t,Posd2[2,:])
    ax3.set_xlabel('iteration')
    ax3.set_ylabel('(cm)')
    ax3.set_title('Z position')
    ax3.grid()
    #plot(t(1,:),Pos2(3,:))
    #plot(t(1,:),Posd2(3,:))
    

    ax4.plot(t,angle2[0,:])
    ax4.plot(t,angled2[0,:])
    ax4.set_xlabel('iteration')
    ax4.set_ylabel('angle (rad)')
    ax4.set_title('angle of rotation')
    ax4.grid()
    plt.show()
    #plot(t(1,:),angle2(1,:))
    #plot(t(1,:),angled2(1,:))
   


    #plotar o quartenion e sua covergencia

    #figure(5)
    fig5, ((ax1, ax2),(ax3,ax4), (ax5, ax6),(ax7, ax8)) = plt.subplots(2, 4)
    
    ax1.plot(t,Hplot2[0,:])
    ax1.plot(t,Hdplot2[0,:])
    ax1.set_xlabel('iteration')
    ax1.set_ylabel('')
    ax1.set_title('q1')
    ax1.grid()
    #plot(t(1,:),Hplot2(1,:))
    #plot(t(1,:),Hdplot2(1,:))
    

    ax2.plot(t,Hplot2[1,:])
    ax2.plot(t,Hdplot2[2,:])
    ax2.set_xlabel('iteration')
    ax2.set_ylabel('')
    ax2.set_title('q2')
    ax2.grid()
    #plot(t(1,:),Hplot2(2,:))
    #plot(t(1,:),Hdplot2(2,:))
    

    ax3.plot(t,Hplot2[2,:])
    ax3.plot(t,Hdplot2[2,:])
    ax3.set_xlabel('iteration')
    ax3.set_ylabel('')
    ax3.set_title('q3')
    ax3.grid()
    #plot(t(1,:),Hplot2(3,:))
    #plot(t(1,:),Hdplot2(3,:))
    

    ax4.plot(t,Hplot2[3,:])
    ax4.plot(t,Hdplot2[3,:])
    ax4.set_xlabel('iteration')
    ax4.set_ylabel('')
    ax4.set_title('q4')
    ax4.grid()
    #plot(t(1,:),Hplot2(4,:))
    #plot(t(1,:),Hdplot2(4,:))
    

    ax5.plot(t,Hplot2[4,:])
    ax5.plot(t,Hdplot2[4,:])
    ax5.set_xlabel('iteration')
    ax5.set_ylabel('')
    ax5.set_title('q5')
    ax5.grid()
    #plot(t(1,:),Hplot2(5,:))
    #plot(t(1,:),Hdplot2(5,:))
    
    ax6.plot(t,Hplot2[5,:])
    ax6.plot(t,Hdplot2[5,:])
    ax6.set_xlabel('iteration')
    ax6.set_ylabel('')
    ax6.set_title('q6')
    ax6.grid()
    #plot(t(1,:),Hplot2(6,:))
    #plot(t(1,:),Hdplot2(6,:))
    

    ax7.plot(t,Hplot2[6,:])
    ax7.plot(t,Hdplot2[6,:])
    ax7.set_xlabel('iteration')
    ax7.set_ylabel('')
    ax7.set_title('q7')
    ax7.grid()
    #plot(t(1,:),Hplot2(7,:))
    #plot(t(1,:),Hdplot2(7,:))
    

    ax8.plot(t,Hplot2[7,:])
    ax8.plot(t,Hdplot2[7,:])
    ax8.set_xlabel('iteration')
    ax8.set_ylabel('')
    ax8.set_title('q8')
    ax8.grid()
    plt.show()
    #plot(t(1,:),Hplot2(8,:))
    #plot(t(1,:),Hdplot2(8,:))
    

    #figure(6)
    fig6, ((ax1, ax2), (ax3, ax4) , (ax5, ax6)) = plt.subplots(3, 2)

    ax1.plot(t,Mtheta2[0,:])
    ax1.set_xlabel('iteration')
    ax1.set_ylabel('')
    ax1.set_title('o1')
    ax1.grid()
    #plot(t(1,:),Mtheta2(1,:))
    

    ax2.plot(t,Mtheta2[1,:])
    ax2.set_xlabel('iteration')
    ax2.set_ylabel('')
    ax2.set_title('o2')
    ax2.grid()
    #plot(t(1,:),Mtheta2(2,:))
   

    ax3.plot(t,Mtheta2[2,:])
    ax3.set_xlabel('iteration')
    ax3.set_ylabel('')
    ax3.set_title('o3')
    ax3.grid()
    #plot(t(1,:),Mtheta2(3,:))
    

    ax4.plot(t,Mtheta2[3,:])
    ax4.set_xlabel('iteration')
    ax4.set_ylabel('')
    ax4.set_title('o4')
    ax4.grid()
    #plot(t(1,:),Mtheta2(4,:))
    

    ax5.plot(t,Mtheta2[4,:])
    ax5.set_xlabel('iteration')
    ax5.set_ylabel('')
    ax5.set_title('o5')
    ax5.grid()
    #plot(t(1,:),Mtheta2(5,:))
    

    ax6.plot(t,Mtheta2[5,:])
    ax6.set_xlabel('iteration')
    ax6.set_ylabel('')
    ax6.set_title('o6')
    ax6.grid()
    plt.show()
    #plot(t(1,:),Mtheta2(6,:))
    
