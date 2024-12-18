from idlelib.parenmatch import CHECK_DELAY
from idlelib.pyparse import C_STRING_FIRST_LINE

import scipy as sp
from sympy import *

# we need C_L, C_D, C_M, alpha, V, gamma, S, S_h, c, l_h, C_Lh, C_Dh, alpha_h, epsilon

C_X = C_L * sin(alpha) - C_D * cos(alpha)
C_Z = - C_L * cos(alpha) - C_D* sin(alpha)


C_Nh = C_Lh*cos(alpha)-C_Dh*sin(alpha) #normal force on horizontal tail
V_h = V #biig assumtions

#C_X0 = - C_D #initial, steady flight condition (the equilibrium situation)
#C_Z0 = - C_L #initial, steady flight condition (the equilibrium situation)
C_X0 = W /(0.5 *rho *V**2 *S) * sin(gamma_0)
C_Z0 = W /(0.5 *rho *V**2 *S) * cos(gamma_0)
C_M0 = 0

C_Xu = 2*C_X0 - diff(C_D,V) * V
C_Zu = 2*C_Z0 - diff(C_L, V)* V
C_Mu = 2*C_M0 - diff(C_M, V)* V

#gliding flight at subsonic speed
C_Xu = -2*C_D
C_Zu = -2*C_L
C_Mu = 0

C_Xalpha = C_L*cos(alpha)+diff(C_L, alpha)*sin(alpha)+C_D*sin(alpha)-diff(C_D, alpha)*cos(alpha)
C_Xalpha = C_L*(1-2*diff(C_L, alpha)/(pi*A*e)) #for alpha=0
C_Zalpha = C_L*sin(alpha)-diff(C_L, alpha)*cos(alpha)-C_D*cos(alpha)-diff(C_D, alpha)*sin(alpha)
C_Zalpha = -diff(C_L, alpha)-C_D #for alpha=0
C_Malpha = diff(C_M, alpha)


C_Xq = 0 #negligible contributions compared to C_Xalpha
C_Mqh = -diff(C_Nh, alpha)*(V_h/V)**2*S_h*l_h**2/(S*c**2)
C_Zqh = -diff(C_Nh, alpha)*(V_h/V)**2*S_h*l_h/(S*c)
C_Zq = 2* (C_Zqh)
C_Mq = 1.1 * C_Zqh*l_h/c

C_Xdiffalpha = 0 #Usually, changes in the airspeed, occur so slowly that this delayed adjustment is not noticeabl
C_Zdiffalpha = -diff(C_Nh, alpha)*(V_h/V)**2*diff(epsilon, alpha)* *S_h*l_h/(S*c)
C_Mdiffalpha = -diff(C_Nh, alpha)*(V_h/V)**2*diff(epsilon, alpha)* *S_h*l_h**2/(S*c**2)

C_Xdeltae = 0 #commonly neglected
C_Zdeltae = #we need C_Nhdeltae
C_Mdeltae = #we need C_Nhdeltae

