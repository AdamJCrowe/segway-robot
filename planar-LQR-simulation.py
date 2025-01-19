import numpy as np
import math
import control
import matplotlib.pyplot as plt
from random import random



# Constants
def getMatrices(const):
    # Intermediate constants
    a = 2 * const['mw'] * pow(const['R'],2) + 2 * const['Iw'] + const['mb'] * pow(const['R'],2)
    b = const['mb'] * const['L'] * const['R']
    c = const['mb'] * pow(const['L'],2) + const['Ib']
    d = const['mb'] * 9.81 * const['L']

    # Matrix entries
    A32 = (d/b) + ((a*c*d) / (pow(b,3) - a*b*c))
    A34 = -(a*d) / (pow(b,2) - (a*c))
    B12 = -(2*c) / (pow(b,2) - a*c)
    B14 = (2*b) / (pow(b,2) - a*c)

    # Matrricies
    A = np.array([[0,1,0,0], [0,0,A32,0], [0,0,0,1], [0,0,A34,0]])
    B = np.array([[0],[B12],[0],[B14]])

    return A, B



# Assess controllability
def assesControl(A,B,n):
    Ctrb = control.ctrb(A,B)        # Calculate controllability matrix 
    result = 0

    # Test if the system is controllable or not
    if (np.linalg.matrix_rank(Ctrb) == n):
        print("Controllability matrix has full rank, so it is possible to control the system.")
    else:
        print("Controllability matrix does not have full rank, so it is not possible to control the system.")
        result = -1

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

    return result



# LQR simulation
def simulation(const, total_time, R, K, A, B):
    # Setup
    loop_time = 1/const['system_freq']                                              # software loop time (s)
    total_loops = int(total_time / loop_time)                                       # total number of software loops for the simulation
    x_0 = np.array([[0],[0],[const['body_initial_angle']*(math.pi/180)],[0]])       # initial state (wheel angle, wheel velocity, body angle, body velocity)
    x_bar = np.array([[const['wheel_target_pos']/R],[0],[0],[0]])                   # target state (wheel angle, wheel velocity, body angle, body velocity)
    n = int(0)                                                                      # how many software loops have passed
    x_n = x_0                                                                       # current state state (wheel angle, wheel velocity, body angle, body velocity)
    x_n1 = x_n                                                                      # estimated state at the time of the next loop
    u = np.array([[0.1]])                                                           # motor torque
    u[0] = 0

    # Results
    sum_error = 0                                                                   # count the total error in the system
    results = np.empty([total_loops+1,4])                                           # record: motor torque, body angle, wheel position, time passed
    results[0,:] = u[0,0], x_n[2,0], x_n[1,0], 0                                    # initial results

    # Simulation
    for n in range (1,total_loops+1):                   
        x_n1 = x_n + (np.dot(A,x_n) + np.dot(B,u))*loop_time                        # system repsonse 
        
        u[0] = -np.dot(K,(x_n - x_bar))                                             # calculate ideal motor torque and ensure it is wihtin limits                     
        if u[0,0] >= const['maximum_torque']:
            u[0] = const['maximum_torque']
        elif u[0,0] <= -const['maximum_torque']:
            u[0] = -const['maximum_torque']
        elif (u[0,0]<const['minimum_torque']) and (u[0,0]>-const['minimum_torque']):
            u[0] = 0
        
        x_n = x_n1                                                                  # update current state 
    
        results[n,:] = u[0,0], x_n[2,0]/(math.pi/180), x_n[0,0]*R, n * loop_time    # record system state and time
        sum_error = sum_error + pow(x_n[2,0],2)             
        if results[n,1] >= 90:
            print("\nThe angle of the robot body is >=90deg, the system has lost control.")
            break

    print("\nTotal body angle error is ", math.sqrt(sum_error))
    return(results)



# Plot results
def plotResults(results, total_time):
    fig, ax = plt.subplots(nrows=1, ncols=3)

    # Body angle plot
    plt.subplot(1, 3, 1)
    plt.plot(results[:,3], results[:,1])
    plt.title("Body angle (deg)") 
    plt.xlabel("Time (s)")
    plt.xlim(xmin=0, xmax=total_time)

    # Wheel angle plot
    plt.subplot(1, 3, 2)
    plt.plot(results[:,3], results[:,2])
    plt.title("Wheel position (m)") 
    plt.xlabel("Time (s)")
    plt.xlim(xmin=0, xmax=total_time)

    # Motor torque plot
    plt.subplot(1, 3, 3)
    plt.plot(results[:,3], results[:,0])
    plt.title("Motor torque (Nm)") 
    plt.xlabel("Time (s)")
    plt.xlim(xmin=0, xmax=total_time)
     
    plt.show()




def main():
    # Simulation constants 
    simulation_constants = {'minimum_torque': 0.08, 'maximum_torque': 0.8, 'system_freq':500, 'body_initial_angle': 20, 'wheel_target_pos':0}
    dynamics_constants = {'mw':0.03, 'R':0.0605, 'mb':0.776, 'Ib':0.0226, 'Iw':0.000055, 'L':0.1708}    # wheel mass, wheel radius, body mass, body mass moment of inertia (units are kg and m)     
    Q_L = np.array([[0.01,0,0,0], [0,0.00002,0,0], [0,0,100,0], [0,0,0,0.0001]])                        # weighted terms (wheel angle, wheel velocity, body angle, body velocity)
    R_L = np.array([[0.5]])                                                                             # weighted term (torque)
    n = 4                                                                                               # number of state variables
    total_time = 20                                                                                     # simulation run time

    # Main code -----------------------------------------------------------------------------------------------------------------------------------------------------------------
    A, B = getMatrices(dynamics_constants)      # Calculate dynamics matrix (A), input matrix (B), length to body's CoM (L)
    
    K, S, E = control.lqr(A, B, Q_L, R_L)       # Calulate state feedback gain matrix (K), solution to Ricatti equation (S), eigenvalues (E)
    K = np.array([[-0.01,-0.001,-1.2,-0.01]])       
    print("The control law is: ",K,"\n")
    
    control_check = assesControl(A,B,n)         # Asses the controllability of the matrix (if -1 is retruned the system can not be controlled)
    
    if control_check == 0:                  
        results = simulation(simulation_constants, total_time, dynamics_constants['R'], K, A, B)    # run the simulation if the system is controllable
        if  results[n,1] < 90:
            plotResults(results, total_time)                                                        # plot the simulation results if the system has not exploded

main()
