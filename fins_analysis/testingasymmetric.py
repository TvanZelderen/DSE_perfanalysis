import numpy as np


C_Ybeta = -0.9896
C_lbeta = -0.0772
C_nbeta = 0.1638
C_Yp = -0.087
C_lp = -0.3444
C_np = -0.0108
C_Yr = 0.43
C_lr = 0.28
C_nr = -0.193
C_Ydeltaa = 0
C_ldetaa = -0.2349
C_ndeltaa = 0.0286
C_Ydeltar = 0.3037
C_ldeltar = 0.0286
C_ndeltar = -0.1261

mu_b = 15.5
K_X2 = 0.012
K_Z2 = 0.037
K_XZ = 0.002
C_L = 1.1360

A = 16*mu_b**3*(K_X2*K_Z2-K_XZ**2)
B = -4*mu_b**2*(2*C_Ybeta*(K_X2*K_Z2-K_XZ**2)+C_nr*K_X2+C_lp*K_Z2+(C_lr+C_np)*K_XZ)
C = 2*mu_b*((C_Ybeta*C_nr-C_Yr*C_nbeta)*K_X2+(C_Ybeta*C_lp-C_lbeta*C_Yp)*K_Z2+((C_Ybeta*C_np-C_nbeta*C_Yp)+(C_Ybeta*C_lr-C_lbeta*C_Yr))*K_XZ+4*mu_b*C_nbeta*K_X2+4*mu_b*C_lbeta*K_XZ+0.5*(C_lp*C_nr-C_np*C_lr))
D = -4*mu_b*C_L*(C_lbeta*K_Z2+C_nbeta*K_XZ)+2*mu_b*(C_lbeta*C_np-C_nbeta*C_lp)+0.5*C_Ybeta*(C_lr*C_np-C_nr*C_lp)+0.5*C_Yp*(C_lbeta*C_nr-C_nbeta*C_lr)+0.5*C_Yr*(C_lp*C_nbeta-C_np*C_lbeta)
E = C_L*(C_lbeta*C_nr-C_nbeta*C_lr)

print(A, B, C, D, E)
coefficients = [A, B, C, D, E]
roots = np.roots(coefficients)
print(roots)

ans = C_lbeta*C_nr-C_nbeta*C_lr
print(ans)