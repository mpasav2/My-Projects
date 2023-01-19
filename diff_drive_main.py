from matplotlib import pyplot as plt
import numpy as np
import control
import math
from scipy import interpolate
from scipy.integrate import odeint

x_loc = np.array([0, 0])
y_loc = np.array([-2, -1])

Qe = np.diag([0.1,0.1,2])
Re = np.diag([0.3,0.3])

def sin(theta):
    return np.sin(theta);

def cos(theta):
    return np.cos(theta)

dirname = os.path.dirname(__file__)
controlfilename = "control.txt"
measurementfilename = "measurement.txt"
timefilename = "time.txt"
statefilename = "state.txt"
abspath = os.path.join(dirname + "/" + controlfilename)
u_all = np.loadtxt(abspath)
abspath = os.path.join(dirname + "/" + measurementfilename)
y_all = np.loadtxt(abspath)
abspath = os.path.join(dirname + "/" + timefilename)
t = np.loadtxt(abspath)


        

self.pause = 0.01
self.fps =20

v=u_all[0]
w=u_all[1]



for b in range(len(t)):
    A = np.array([[(v*np.cos(theta)),0,0],[0,v*np.sin*(theta),0],[0, 0, w],])

    C = np.array([[0,0,1,0],[0,0,0,1]])
    #compute eigenvalues of uncontrolled system
    eigVal,eigVec = np.linalg.eig(A)
    print('eig-vals (uncontrolled)=') #eigvalues on imaginary axis
    for i in np.arange(0,4):
        print(eigVal[i])

    #compute observability of the system 
    Ob = control.obsv(A,C)
    print('rank=',np.linalg.matrix_rank(Ob))
    Ob_alt = control.ctrb(np.transpose(A),np.transpose(C)) #alternate way of computing observability

    

    #pole placement
    print('\nPole placement');
    p = np.array([-5,-5.5,-6.5,-6])
    L_trans = control.place(np.transpose(A),np.transpose(C),p)
    L = np.transpose(L_trans)
    print("L = ",L)
    eigVal,eigVec = np.linalg.eig(np.subtract(A,np.matmul(L,C)))
    print('eig-vals (controlled)=')
    for i in np.arange(0,4):
        print(eigVal[i])


#Design Kalman filter (Stochastic system)
L, P, E = control.lqe(parms.A, parms.G, parms.C, Qe, Re)
print("Kalman Gain: L = ",L)

x0 = np.array([0.5,0,0,0])
x0est = np.array([0.2,0,0,0])
x0big = np.concatenate((x0, x0est))

t0 = 0;
tend = 5;

Npts = 101
N = 4
shape = (Npts,2*N)
t = np.linspace(t0, tend, Npts)
xbig = np.zeros(shape)
for i in range(0,2*N):
    xbig[0,i] = x0big[i];

for i in range(0,Npts-1):
    f1_disturb = np.random.normal(f1_mean,f1_dev)
    f2_disturb = np.random.normal(f2_mean,f2_dev)
    q1dot_sensor = np.random.normal(q1dot_mean,q1dot_dev)
    q2dot_sensor = np.random.normal(q2dot_mean,q2dot_dev)
    t_temp = np.array([t[i], t[i+1]])
    physical_parms = (parms.A,parms.C,parms.G)
    control_parms = (L,f1_disturb,f2_disturb,q1dot_sensor,q2dot_sensor)
    all_parms = physical_parms + control_parms
    xbig_temp = odeint(spring_mass_rhs, x0big, t_temp, args=all_parms)
    for j in range(0,2*N):
        x0big[j] = xbig_temp[1,j]
        xbig[i+1,j] = x0big[j]

plt.figure(2)
plt.plot(y_all[:,0],y_all[:,1],'k',alpha=0.2)
plt.plot(x_loc,y_loc,color='black',linewidth=5);
plt.grid()
plt.ylabel("y (measured)")
plt.xlabel("x (measured)")
plt.gca().set_aspect('equal')
# plt.show()
plt.show(block=False)
plt.pause(50)
plt.close()
