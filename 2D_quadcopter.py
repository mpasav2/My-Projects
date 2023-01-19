import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import matplotlib.pyplot as plt
import os
import control
import sympy as sp
from scipy import interpolate
from scipy.integrate import odeint
import scipy.linalg

xml_path = 'bicopter.xml' #xml file (assumes this is in the same folder as this file)
simend = 5 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

np.random.seed(1)
l = 0.8;
mtot = 1.00
g = 9.81
I = 0.1
pos_lim = 0.1
phi_lim = 0.1
C = np.zeros((3,6))
C[0][0] = 1
C[1][1] = 1
C[2][5] = 1


#Set flag_estimator = 0 #and test LQR controller
#2) Set flag_estimator = 1 and test LQE/Kalman filter
flag_estimator = 0
a_dev = 5;
alpha_dev = 20;
pos_dev = 0.1
rate_dev = 0.1

def sin(theta):
    return np.sin(theta)

def cos(theta):
    return np.cos(theta)

def measurement():
    x_measured = data.qpos[0]+np.random.normal(0,pos_dev)
    y_measured = data.qpos[1]+np.random.normal(0,pos_dev)
    phidot_measured = data.qvel[2]+np.random.normal(0,rate_dev)
    y = np.array([x_measured,y_measured,phidot_measured])
    return y
#print(y)
#y1= measurement(0.1,0.1)
#x=y[1]
#y=y[2]
#phi_dott=y[3]
#def matrix(A,B):
    
    #x1=x
    #x2=y
    #x3=phi
    #x4=x_dott
    #x5=y_dott
    #x6=phi_dott
    #us = mtot*g
    #ud = 0
    #x=[x1,x2,x3,x4,x5,x6]
    #x1,x2,x3,x4,x5,x6 = sp.symbols("x1 x2 x3 x4 x5 x6")
    #f_x_u =[x4,x5,x6,(-us*np*sin(x3))/m,(us*np.cos(x3))/(m-g),0.5*ud*l*I]
    #A is the df/dx
    
    #A=[[x_ddot,0,0,1,0,0],[0,y_ddot,0,0,1,0],[0,0(-us*np.cos(x3))/m,0,0,0],[0,0,(-us*np.son(x3))/(mtot-g),0,0,0],[0,0,0,0,0,0]]
#A=[[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,-g,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
#B=[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,1/mtot,0],[0,0,(0.5*l)/I]]
A=[[0,0,-g],[0,0,0,],[0,0,0]]
B=[[0,0],[0,0],[0,(0.5*l)/I]]

    #x_dot=f(x,u)+(g*w)



def initialize(model,data):

    #pass

    global K, L
    #HINT1 LQR: Do this first
    # a) Design LQR gain K here.
    # b) Set flag_estimator = 0 (top of the file). Full state estimate is available
    # c) Code the controller in the function "controller" and run
    #p= numpy.array([-1,-2,-3,-4,-5,-6])
    #K= control.place(A,B,p)

    #eigen1,eigVec1=numpy.linalg.eig(numpy.subtract(A,numpy.matmul(B,K)))
    ##print("k",K)
    #print("eigen1",eigen1)
    


    #
    #HINT2 LQR: Do this after LQR has been 
    # #first, try to solve the ricatti equation
    #Q = np.eye((3))
    #R = np.array([[-1,4+2j],[3,8+1j]]);
    print(A,B,Q,R)
    Q = np.diag([1,1,1,1,1,1])
    R = np.diag([2e-2,2e-2]);
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
        #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(np.matrix(B.T)*np.matrix(X)))

    eigVals, eigVecs = scipy.linalg.eig(A-B*K)

    return K, X, eigVals
    # a) Design observer/filter gain L next.
    # b) Set flag_estimator = 1 (top of the file), Only x,y,phidot is measured.
    # c) Code the estimator in the function "estimator" and run


def estimator(model,data,x_estimate,u):

    global L
    y = measurement()
    #use y and L to estimate the state: x_estimate
    #x_estimate is then returned by this function

    return x_estimate

def controller(model, data,x):
    #put the controller here. This function is called inside the simulation.
    #pass

    global K

    #this is the nominal control
    us = mtot*g
    ud = 0




    #apply disturbance to test the controller (dont remove/modify these lines)
    fx_d = np.random.normal(0,a_dev)
    fz_d = np.random.normal(0,a_dev)
    tau_d = np.random.normal(0,alpha_dev)
    data.qfrc_applied[0] = fx_d
    data.qfrc_applied[1] = fz_d
    data.qfrc_applied[2] = tau_d

    #This code applies control to the bicopter (dont remove/modify these lines)
    phi = data.qpos[2];
    body = 1 #1 is bicopter (0 is world)
    data.xfrc_applied[body][0] = -us*sin(phi)
    data.xfrc_applied[body][2] = us*cos(phi)
    data.xfrc_applied[body][4] = 0.5*l*ud

    u = np.array([us,ud])
    return u;

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = 89.26731438394698 ; cam.elevation = -5.7007952873653265 ; cam.distance =  3.035328453847003
cam.lookat =np.array([ 0.0 , 0.0 , 0 ])

time = []
x = []
y = []
phi = []
xdot = []
ydot = []
phidot = []
us = []
ud = []

#initialize the controller
initialize(model,data)

x_estimate = np.array([0,0,0,0,0,0])
u = np.array([mtot*g,0])

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        if (flag_estimator==1):
            x_estimate = estimator(model,data,x_estimate,u)
        else:
            x_estimate = np.array([data.qpos[0],data.qpos[1],data.qpos[2],\
                                   data.qvel[0],data.qvel[1],data.qvel[2],])
        u = controller(model,data,x_estimate)
        mj.mj_step(model, data)

    time.append(data.time)
    x.append(data.qpos[0])
    y.append(data.qpos[1])
    phi.append(data.qpos[2])
    xdot.append(data.qvel[0])
    ydot.append(data.qvel[1])
    phidot.append(data.qvel[2])
    us.append(u[0])
    ud.append(u[1])


    if (data.time>=simend):
        plt.figure(1)
        plt.subplot(3,1,1)
        plt.plot(time,x,'r')
        plt.plot(time,pos_lim*np.ones(len(time)),'k-.');
        plt.plot(time,-pos_lim*np.ones(len(time)),'k-.');
        plt.ylabel("x")
        plt.subplot(3,1,2)
        plt.plot(time,y,'r')
        plt.plot(time,pos_lim*np.ones(len(time)),'k-.');
        plt.plot(time,-pos_lim*np.ones(len(time)),'k-.');
        plt.ylabel("y")
        plt.subplot(3,1,3)
        plt.plot(time,phi,'r')
        plt.plot(time,phi_lim*np.ones(len(time)),'k-.');
        plt.plot(time,-phi_lim*np.ones(len(time)),'k-.');
        plt.ylabel("phi")

        # plt.show()
        plt.show(block=False)
        plt.pause(3)
        plt.close()

        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    #cam.lookat[0] = data.qpos[0]
    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
