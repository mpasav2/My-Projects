import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3


def cos(angle):
    return np.cos(angle)

def sin(angle):
    return np.sin(angle)

def DH(a,alpha,d,theta):

    cth = cos(theta);
    sth = sin(theta);
    cal = cos(alpha);
    sal = sin(alpha);

    H_z_theta = np.array([
                   [cth, -sth, 0, 0],
                   [sth,  cth, 0, 0],
                   [0 ,    0, 1, 0],
                   [0 ,    0, 0, 1]]);

    H_z_d = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, d],
                [0, 0, 0, 1]]);

    H_x_a = np.array([
                    [1, 0, 0, a],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]);

    H_x_alpha = np.array([
                  [1,   0,   0,   0],
                  [0,  cal, -sal, 0],
                  [0,  sal,  cal, 0],
                  [0,   0,    0,  1]]);

    H_z = np.matmul(H_z_theta,H_z_d);
    H_x = np.matmul(H_x_a,H_x_alpha);
    H = np.matmul(H_z,H_x);
    # H = H_z_theta*H_z_d*H_x_a*H_x_alpha;

    return H

a1 = 0; alpha1 = np.pi/2; d1=0.089159; theta1 = 0;
H01 = DH(a1,alpha1,d1,theta1); #H^0_1

a2 = -0.425; alpha2 = 0; d2=0; theta2 = 0;
H12 = DH(a2,alpha2,d2,theta2); #H^1_2

a3 = -0.39225; alpha3 = 0; d3=0; theta3 = 0;
H23 = DH(a3,alpha3,d3,theta3); #H^2_3

a4 = 0; alpha4 = np.pi/2; d4=0.10915; theta4 = 0;
H34 = DH(a4,alpha4,d4,theta4);

a5 = 0; alpha5 = -np.pi/2; d5=0.09465; theta5 = 0;
H45 = DH(a5,alpha5,d5,theta5);

a6 = 0; alpha6 = 0; d6=0.0823; theta6 = 0;
H56 = DH(a6,alpha6,d6,theta6);



#%Location of joint 1
endOfLink1 = H01[0:3,3];
# print(endOfLink1)

#Location of joint 2
H02 = np.matmul(H01,H12);
endOfLink2 = H02[0:3,3];
# print(endOfLink2)

#Location of joint 3
H03 = np.matmul(H02,H23); #H01*H12*H23;
endOfLink3 = H03[0:3,3];
# print(endOfLink3)

#Location of joint 4
H04 = np.matmul(H03,H34); #H01*H12*H23;
endOfLink4 = H04[0:3,3];

#Location of joint 5
H05 = np.matmul(H04,H45); #H01*H12*H23;
endOfLink5 = H05[0:3,3];

#Location of joint 6
H06 = np.matmul(H05,H56); #H01*H12*H23;
endOfLink6 = H06[0:3,3];





#end-effector position and orientation.
position_of_end_effector = H06[0:3,3];
orientation_of_end_effector = H06[0:3,0:3];
print(position_of_end_effector)
print(orientation_of_end_effector)

fig = plt.figure()
ax = p3.Axes3D(fig)

line1, = ax.plot([0, endOfLink1[0]],[0, endOfLink1[1]],[0, endOfLink1[2]], color='red', linewidth=2)
line2, = ax.plot([endOfLink1[0], endOfLink2[0]],[endOfLink1[1], endOfLink2[1]],[endOfLink1[2], endOfLink2[2]],
                  color='blue', linewidth=2)
line3, = ax.plot([endOfLink2[0], endOfLink3[0]],[endOfLink2[1], endOfLink3[1]],[endOfLink2[2], endOfLink3[2]],
                  color='lightblue', linewidth=2)
line4, = ax.plot([endOfLink3[0], endOfLink4[0]],[endOfLink3[1], endOfLink4[1]],[endOfLink3[2], endOfLink4[2]],
                  color='brown', linewidth=2)
line5, = ax.plot([endOfLink4[0], endOfLink5[0]],[endOfLink4[1], endOfLink5[1]],[endOfLink4[2], endOfLink5[2]],
                  color='green', linewidth=2)
line6, = ax.plot([endOfLink5[0], endOfLink6[0]],[endOfLink5[1], endOfLink6[1]],[endOfLink5[2], endOfLink6[2]],
                  color='black', linewidth=2)

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.view_init(azim=64,elev=29)
plt.show()
# plt.show(block=False)
# plt.pause(2)
# plt.close()
