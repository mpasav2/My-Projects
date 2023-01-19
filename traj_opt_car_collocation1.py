from matplotlib import pyplot as plt
import numpy as np
import math
from scipy import interpolate
from scipy.integrate import odeint
import random
import scipy.optimize as opt

class parameters:
    def __init__(self):
        self.D = 5
        self.N = 12

def cost(x):
    return x[0]

def nonlinear_fn(x):
    parms = parameters()
    D = parms.D
    N = parms.N

    T = x[0];
    t = np.linspace(0,T,N+1)
    dt = t[1]-t[0];
    pos = [0]* (N+1)
    vel = [0]* (N+1)
    u = [0] * (N+1)
    for i in range(0,N+1):
        pos[i] = x[i+1]
        vel[i] = x[i+1+N+1]
        u[i] = x[i+1+N+1+N+1]

    defect_pos = [0]* N
    defect_vel = [0]* N
    for i in range(0,N):
        defect_pos[i] = pos[i+1] - pos[i] - vel[i]*dt;
        defect_vel[i] = vel[i+1] - vel[i] - 0.5*(u[i]+u[i+1])*dt;

    ceq = [0] * (2*N+4)
    ceq[0] = pos[0]
    ceq[1] = vel[1]
    ceq[2] = pos[N] - D
    ceq[3] = vel[N]
    for i in range(0,N):
       ceq[i+4] = defect_pos[i]
       ceq[i+N+4] = defect_vel[i]

    return ceq


random.seed(1)
parms = parameters()
N = parms.N
D = parms.D

T_min = 1; T_max = 5;
pos_min = 0; pos_max = D;
vel_min = -10; vel_max = 10;
u_min = -5; u_max = 5;

T_opt = 2
pos_opt = [0] * (N+1)
vel_opt = [0] * (N+1)
u_opt = [0] * (N+1) #initialize u to zeros

x0 = [0] * (1+N+1+N+1+N+1)
x_min = [0] * (1+N+1+N+1+N+1)
x_max = [0] * (1+N+1+N+1+N+1)
x0[0] = T_opt
x_min[0] = T_min
x_max[0] = T_max
k = 0
for i in range(1,1+N+1):
    x0[i] = pos_opt[k]
    x_min[i] = pos_min
    x_max[i] = pos_max
    k+=1

k = 0
for i in range(1+N+1,1+N+1+N+1):
    x0[i] = vel_opt[k]
    x_min[i] = vel_min
    x_max[i] = vel_max
    k +=1

k = 0
for i in range(1+N+1+N+1,1+N+1+N+1+N+1):
    x0[i] = u_opt[k]
    x_min[i] = u_min
    x_max[i] = u_max
    k +=1


limits = opt.Bounds(x_min,
                    x_max)

eq_cons = {'type': 'eq',
           'fun' : nonlinear_fn
           }

res = opt.minimize(cost, x0, method='SLSQP',
               constraints=[eq_cons], options={'ftol': 1e-12, 'disp': True, 'maxiter':500},
               bounds=limits)
#
print(x0)
x = res.x
print(x)
print(res.success)
print(res.message)
print(res.status)


pos = [0]* (N+1)
vel = [0]* (N+1)
u = [0] * (N+1)
for i in range(0,N+1):
    pos[i] = x[i+1]
    vel[i] = x[i+1+N+1]
    u[i] = x[i+1+N+1+N+1]

T = x[0];
t = np.linspace(0,T,N+1)
print(pos)
print(vel)
print(u)


plt.figure(1)
plt.subplot(3,1,1)
plt.plot(t,pos)
plt.ylabel("x")
plt.subplot(3,1,2)
plt.plot(t,vel)
plt.ylabel("xdot")
plt.subplot(3,1,3)
plt.plot(t,u)
plt.xlabel("t")
plt.ylabel("u")
# plt.show()
plt.show(block=False)
plt.pause(10)
plt.close()
