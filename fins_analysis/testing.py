import numpy as np


mu_c = 102.7
K_Y2 = 0.98
C_Zdiffalpha = -1.43
C_Mdiffalpha = -3.7
C_Zq = -3.86
C_Mq = -7.04
C_Xu = -0.2199
C_Zu = -2.272
C_Mu = 0
C_Zalpha = -5.16
C_Malpha = -0.43
C_Xalpha = 0.4653
C_X0 = 0
C_Z0 = -1.1360


A = 4*mu_c**2*K_Y2*(C_Zdiffalpha-2*mu_c)
B = C_Mdiffalpha*2*mu_c*(C_Zq+2*mu_c)-C_Mq*2*mu_c*(C_Zdiffalpha-2*mu_c)-2*mu_c*K_Y2*(C_Xu*(C_Zdiffalpha-2*mu_c)-2*mu_c*C_Zalpha)
C = C_Malpha*2*mu_c*(C_Zq+2*mu_c)-C_Mdiffalpha*(2*mu_c*C_X0+C_Xu*(C_Zq+2*mu_c))+C_Mq*(C_Xu*(C_Zdiffalpha-2*mu_c)-2*mu_c*C_Zalpha)+2*mu_c*K_Y2*(C_Xalpha*C_Zu-C_Zalpha*C_Xu)
D = C_Mu*(C_Xalpha*(C_Zq+2*mu_c)-C_Z0*(C_Zdiffalpha-2*mu_c))-C_Malpha*(2*mu_c*C_X0+C_Xu*(C_Zq+2*mu_c))+C_Mdiffalpha*(C_X0*C_Xu-C_Z0*C_Zu)+C_Mq*(C_Xu*C_Zalpha-C_Zu*C_Xalpha)
E = -C_Mu*(C_X0*C_Xalpha+C_Z0*C_Zalpha)+C_Malpha*(C_X0*C_Xu+C_Z0*C_Zu)

print(A, B, C, D, E)
coefficients = [A, B, C, D, E]
roots = np.roots(coefficients)
print(roots)



M = [[C_Xu-2*mu_c, C_Xalpha, C_Z0, 0],
     [C_Zu,C_Zalpha+(C_Zdiffalpha-2*mu_c), -C_X0, C_Zq+2*mu_c],
     [0,0,-1,1],
     [C_Mu,C_Malpha+C_Mdiffalpha,0,C_Mq-2*mu_c*K_Y2]]
#print(np.linalg.eigvals(M))