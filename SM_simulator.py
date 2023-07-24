import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Parameters of simulator
maxFreq = 200.0 # frequency of stepper motor are changed linearly from 0 to maxFreq, 1/s
time = 20.0     # seconds
numPoints = 200000

# Mechanical parameters of stepper motor
N = 200         # number of phases of stepper motor
J = 570E-07     # Inertia moment, kg*m^2
K = 31.0E-02*9.8/2.8    # Ratio of maximum torque to maximum phase current N*m/A
PhiTol = 0.09*np.pi/180.0 # tolerqnce, radian
DT = 0.09       # Detent torque, N*m
FT = 0.02       # Friction torque, N*m

# Electrical parameters
Iref = 2.8      # Phase current, A
Vpow = 24.0     # Power voltage of phases of stepper motor
L = 6.8E-03     # Phase inductance, Hn
R = 1.5         # Phase resistance, Ohm

def Iref1(t):
    freq = maxFreq/time*t
    timeQuant = int(t * freq) % 4
    return Iref if timeQuant < 2 else -Iref

def Iref2(t):
    freq = maxFreq/time*t
    timeQuant = int(t * freq) % 4
    return Iref if timeQuant > 0 and timeQuant < 3 else -Iref

def GetVpow(t, iref, I):
    if iref > 0.0:
        if I < iref:
            return Vpow
        elif I - iref > 0.1:
            return -Vpow
        else:
            return I*R
    else:
        if I > iref:
            return -Vpow
        elif iref - I > 0.1:
            return Vpow
        else:
            return I*R

def dIdt(t, iref, I):
    return (GetVpow(t, iref, I) - I*R)/L

def ElectricTorque(phi, I1, I2):
    return K*(I1*np.sin(N*phi) - I2*np.cos(N*phi))

def DetentTorque(phi):
    return DT*np.sin(N*phi*4.0)

def SumTorque(phi, omega, I1, I2):
    powerTorque = ElectricTorque(phi, I1, I2) - DetentTorque(phi)
    friqTorque = FT
    if friqTorque > abs(powerTorque):
        friqTorque = abs(powerTorque)
    return powerTorque - friqTorque*np.sign(omega)

prevPercentage = 0;
"""
y has format:
[phi(t), omega(t), I1(t), I2(t)],
where:
 0. phi(t) - angle of rotor of stepper motor
 1. omega(t) - angle velocity  of rotor of stepper motor
 2. I1(t) - current of first phase
 3. I2(t) - current of second phase
"""
def MovementEquation(t, y):
    global prevPercentage
    percentage = int(t/time*100.0)
    if percentage > prevPercentage:
        prevPercentage = percentage
        print("{0}%".format(percentage))
    return [y[1], SumTorque(y[0], y[1], y[2], y[3])/J, dIdt(t, Iref1(t), y[2]), dIdt(t, Iref2(t), y[3])]

y0 = [0.0, 0.0, 0.0, 0.0]
sol = solve_ivp(MovementEquation, [0.0, time], y0, vectorized = True, dense_output = True)
t = np.linspace(0, time, numPoints)
z = sol.sol(t)
plt.plot(t, z[0])
plt.xlabel("time")
plt.legend(["phi", "phi"], shadow = True)
plt.title("Stepper motor movement simulation")
plt.show()
