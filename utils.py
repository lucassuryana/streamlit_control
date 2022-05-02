# PID based cruise control
# @Lucas Elbert Suryana
# Adapted from Matlab code - Meng Wang
# 2 May, 2022
import numpy as np
import matplotlib.pyplot as plt

def simulation(kp, ki, kd, saturation = 1):
    tau = 0.2 # Actuator delay [s]
    v0 = 29 # Initial speed [m/s]
    T = 30 # Simlation period [s]
    dt = 0.1 # Simulation time step [s]
    m = T/dt

    saturation = 1 # whehter impose saturation in control input
    umax = 2 # Maximum acceleration [m/s^2]
    umin = -2 # Minimum acceleration [m/s^2]

    # Initialisation of variables
    ep = np.zeros(int(m)) # Proportional
    ed = np.zeros(int(m)) # Derivative
    ei = np.zeros(int(m)) # Integral
    u = np.zeros(int(m)) # Control input
    v = np.zeros(int(m)) 
    v[0] = v0 # Initial speed
    a = np.zeros(int(m))  # Initial acceleration
    a[0] = 0 # Initial acceleration
    vref = np.zeros(int(m))
    vref[0] = v0 # Reference speed [m/s]
    time = np.arange(0,T,dt) # Time

    # The main simulation loop
    for i in range(0,int(m)): 
        # Reference speed
        if dt*i >= 5:
            vref[i] = 30
        else:
            vref[i] = v0
        
        # Calculate errors
        ep[i] = vref[i] - v[i]
        if i == 0:
            ed[i] = ep[i]
        else: 
            ed[i] = (ep[i] - ep[i-1])/dt
            
        ei[i] = sum(ep)
        
        # Control algorithm
        u[i] = kp*ep[i] + kd*ed[i]+ ki*ei[i]
        
        if saturation == 1:
            u[i] = min(u[i], umax)
            u[i] = max(u[i], umin)
        
        # System dynamics model:
        # da/dt = (u -a)/tau
        if i != int(m-1):
            a[i+1] = dt*(u[i] - a[i])/tau + a[i] 
            v[i+1] = dt*a[i] + v[i]
        else:
            np.append(a, dt*(u[i] - a[i])/tau + a[i])
            np.append(v, dt*a[i] + v[i])
    
    return a, v, u, vref, time

def if_cc():
    st.write("In this section we will learn to simulate a cruise control with PID controller for a system with:")
    st.write("- State variable: speed $(v)$ and acceleration $(a)$")
    st.write("- Control input $(u)$")
    st.write("- System dynamics:")
    st.latex(r'''
    \frac{dv(t)}{dt} = a(t)
    ''')
    st.latex(r'''
    \frac{da(t)}{dt} = \frac{u(t)-a(t)}{c}
    ''')
    st.write("where $c$ is the delay of actuator with value of 0.2")

    st.write("The control objective of this system is to maintain a reference speed $v_{ref}$ by implementing PID controller:")
    st.latex(r'''
     PID =
     u(t) =
     K_{p}e(t) + K_{i} \int_{0}^{t} e(\tau) d\tau + K_{d}\frac{de(t)}{dt}
     ''')
    st.write("where:")
    st.latex(r'''
     e(t) = v_{ref}(t) - v(t)
     ''')
    st.latex(r'''
     u(t) \in [-2, 2]
     ''')

    st.write("Setting in the simulation:")
    st.write("- Simulation period: 30 $s$")
    st.write("- Reference speed changes from 29 to 30 $m/s$ from 5$s$ onward")
    st.write("- Initial speed 29 $m/s$")
    st.write("- Initial values of $K_{p}, K_{i}, K_{d}$ are respectively: 1, 0.01, 0.1")
