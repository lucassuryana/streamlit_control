import matplotlib.pyplot as plt
import plotly.express as px
import streamlit as st
import pandas as pd
from utils import *


#Description
sidebar = st.sidebar
sidebar.title('Control Scheme')
sidebar.write(
"""
Please select the simulation you intend to do.
"""
)
button = sidebar.radio("What kind of simulation?", ('CC','ACC')) 

if button =='CC':
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

    col1, col2, col3 = st.columns(3)
    with col1:
        kp = float(st.text_input('Kp:', 1))
    with col2:
        ki = float(st.text_input('Ki:', 0.01))
    with col3:
        kd = float(st.text_input('Kd:', 0.1))

    sim = st.button('Run simulation!')
    if sim == True:
        a, v, u, vref, time = simulation(kp, ki, kd)
        df = pd.DataFrame(dict(
            a = a,
            v = v,
            u = u,
            vref = vref,
            time = time
            ))

        fig, axs = plt.subplots(2, dpi=50)
        fig.tight_layout(pad=3.0)
        axs[0].plot(time, a, label = 'Actual acceleration')
        axs[0].plot(time, u, label = 'Control input')
        axs[0].set_xlabel('$time (s)$')
        axs[0].set_ylabel('$acceleration (m/s^{2})$')
        axs[0].grid()
        axs[0].legend()

        axs[1].plot(time, v*3.6, label = 'Actual speed')
        axs[1].plot(time, vref*3.6, label = 'Reference speed')
        axs[1].set_xlabel('$time (s)$')
        axs[1].set_ylabel('$speed (km/h)$')
        axs[1].grid()
        axs[1].legend()

        st.pyplot(fig)   

else:
    col8, col9 = st.columns(2)
