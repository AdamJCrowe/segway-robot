import numpy as np
import math
import control
import matplotlib.pyplot as plt
from random import random



### Simulation inputs ###

# Simulaton settings
body_initial_angle = 20         # initial angle of the robot body (deg)
wheel_target_pos = 0            # target X position of the wheel (m)
minimum_torque = 0.08           # minimum motor torque (Nm)
maximum_torque = 0.8            # maximum motor torque (Nm)
system_freq = 500               # frequency the microcontroller runs at (Hz)
lag = 0.003                    # lag between the actualy position of the system and the motor torque being updated based on that   
gyroscope = True               # does the gyroscope work
test_robustness = True          # option to test robustness of the system my adjusting inertia terms after calculating LQR's K matrix
total_time = 10                 # simulation length (s)
LQR_Q = np.array([[0.01,0,0,0], [0,0.00002,0,0], [0,0,100,0], [0,0,0,0.0001]])   # weighted terms (wheel angle, wheel velocity, body angle, body velocity)
LQR_R = np.array([[0.5]])                                                         # weighted term (torque)

# Inertia terms
mw = 0.03                       # wheel mass (kg)
R = 0.0605                      # wheel radius (m)
mb = 0.776                      # body mass (kg
Ib = 0.0226                     # body moment of inertia (kgm^2)



### Constants ###
def getConstants():
    Iw = 0.5 * mw * pow((R),2)  # wheel moment of inertia
    L = math.sqrt(Ib/mb)        # distance from wheel axis to body's CoM

    # Intermediate constant
    a = 2*mw*pow(R,2) + 2*Iw + mb*pow(R,2)
    b = mb*L*R
    c = mb*pow(L,2) + Ib
    d = mb*9.81*L
    A32 = (d/b) + ((a*c*d) / (pow(b,3) - a*b*c))
    A34 = -(a*d) / (pow(b,2) - (a*c))
    B12 = -(2*c) / (pow(b,2) - a*c)
    B14 = (2*b) / (pow(b,2) - a*c)
    n=4

    # Matrricies
    A = np.array([[0,1,0,0], [0,0,A32,0], [0,0,0,1], [0,0,A34,0]])
    B = np.array([[0],[B12],[0],[B14]])

    return A, B, n, L

# Calculate constants
A, B, n, L = getConstants()
K, S, E = control.lqr(A, B, LQR_Q, LQR_R) # K is state feedback gains, S is solution to Ricatti equation and E eigenvalues

# Test robustness by including model uncertanity
if (test_robustness == True):
    mw = mw * 1                    
    R = R * 1
    mb = mb * 1                 
    Ib = Ib * 1
    maximum_torque = maximum_torque * 1
    system_freq = system_freq * 1
    lag = lag * 1
    A, B, n, L = getConstants()



### Asses controllability ###

# Controlability matrix 
Ctrb = control.ctrb(A,B)

# Test if the system is controllable or not
if (np.linalg.matrix_rank(Ctrb) == n):
    print("Controllability matrix has full rank, so it is possible to control the system.")
else:
    print("Controllability matrix does not have full rank, so it is not possible to control the system.")

# Construct gramian matrix
CtrbT = np.transpose(Ctrb)
Gram = np.dot(Ctrb,CtrbT)
for i in range(n):
    for j in range(n):
        Gram[i, j] = Gram[i, j] / 1000000 # adjust to allow SVD calcs

# Find SVD of gram matrix       sanity check - U*S*VT should equal Gram  - sometimes this does not work if some gram matrix terms are very large or small
U, S, V = np.linalg.svd(Gram)
VT = np.transpose(V)
sigma = np.zeros((n, n))
for i in range(n):
    sigma[i, i] = S[i]

# Asses controllability quantatively
USVT = np.dot(U, np.dot(sigma,VT))
if (np.allclose(Gram, USVT) == True):       # USVT will be used for a sanity check that SVD was calculated correctly
    print("\nCondition number is (where smaller = more controllable):", np.linalg.cond(sigma))
else:
    print("\nThe U*S*VT matrix does not equal the gram matrix, the SVD has been done incorrectly") # sanity check that SVD was done correctly



### LQR simulation ###

# Intermediate constants                              
loop_time = 1/system_freq                   # software loop time (s)

# Encoder feedback?
if gyroscope == False:
    K[0,3] = 0

# Simulation variables
x_0 = np.array([[0],[0],[body_initial_angle*(math.pi/180)],[0]])        # initial state (wheel angle, wheel velocity, body angle, body velocity)
x_bar = np.array([[wheel_target_pos/R],[0],[0],[0]])        # target state (wheel angle, wheel velocity, body angle, body velocity)
n = int(0)                                  # how many software loops have passed
total_loops = int(total_time / loop_time)   # total number of software loops for the simulation
x_n = x_0                                   # current state state (wheel angle, wheel velocity, body angle, body velocity)
x_n1 = x_n                                  # estimated state at the time of the next loop
u = np.array([[0.1]])                       # torque
u[0] = 0

sum_error = 0                               # count the total error in the system

# Variables to plot the results
results_torque = np.empty(total_loops+1)
results_body_angle = np.empty(total_loops+1)
results_wheel_pos = np.empty(total_loops+1)
results_time = np.empty(total_loops+1)
# Initial results
results_torque[0] = u[0,0]
results_body_angle[0] = x_n[2,0]
results_wheel_pos[0] = x_n[1,0]
results_time[0] = 0

# Simulation
for n in range (1,total_loops+1):
    x_n1 = x_n + (np.dot(A,x_n) + np.dot(B,u))*lag      # system moves before updating motor torque
    
    u[0] = -np.dot(K,(x_n - x_bar))                                     # calculate the current motor torque                     
    if u[0,0] >= maximum_torque:
        u[0] = maximum_torque
    elif u[0,0] <= -maximum_torque:
        u[0] = -maximum_torque
    elif (u[0,0]<minimum_torque) and (u[0,0]>-minimum_torque):
        u[0] = 0

    x_n1 = x_n1 + (np.dot(A,x_n) + np.dot(B,u))*(loop_time-lag)      # system moves after updating motor torque
    
    x_n = x_n1                                                      # current state is updated

    #Uupdate the results arrays
    results_torque[n] = u[0,0]
    results_body_angle[n] = x_n[2,0]/(math.pi/180)
    results_wheel_pos[n] = x_n[0,0]*R
    results_time[n] = n * loop_time

    sum_error = sum_error + pow(x_n[2,0],2)

    if results_body_angle[n] >= 90:
        print("\nThe angle of the robot body is >=90deg, the system has lost control.")
        break



print("\nTotal body angle error is ", math.sqrt(sum_error))

### Plot the results ###
if  results_body_angle[n] < 90:
    fig, ax = plt.subplots(nrows=1, ncols=3)

    # Body angle plot
    plt.subplot(1, 3, 1)
    plt.plot(results_time, results_body_angle)
    plt.title("Body angle (deg)") 
    plt.xlabel("Time (s)")
    plt.xlim(xmin=0, xmax=total_time)

    # Wheel angle plot
    plt.subplot(1, 3, 2)
    plt.plot(results_time, results_wheel_pos)
    plt.title("Wheel position (m)") 
    plt.xlabel("Time (s)")
    plt.xlim(xmin=0, xmax=total_time)

    # Torque plot
    plt.subplot(1, 3, 3)
    plt.plot(results_time, results_torque)
    plt.title("Motor torque (Nm)") 
    plt.xlabel("Time (s)")
    plt.xlim(xmin=0, xmax=total_time)
     
    plt.show()

