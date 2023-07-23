import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Parameters of simulator
maxFreq = 10.0 # frequency of stepper motor are changed linearly from 0 to maxFreq, 1/s
time = 10.0     # seconds
# Mechanical parameters
N = 200         # number of phases of stepper motor
J = 570E-07     # Inertia moment, kg*m^2
K = 31.0E-02/2.8    # Ratio of maximum torque to maximum phase current
PhiTol = 0.09*np.pi/180.0
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

def PowTorque(phi, I1, I2):
    return K*(I1*np.sin(N*phi) - I2*np.cos(N*phi))

def dI1dt(t, I1):
    iref = Iref1(t)
    if abs(I1 - iref) < 0.1:
        return 0.0
    else:
        return (Vpow*np.sign(iref) - I1*R)/L

def dI2dt(t, I2):
    iref = Iref2(t)
    if abs(I2 - iref) < 0.1:
        return 0.0
    else:
        return (Vpow*np.sign(iref) - I2*R)/L

maxFrictionTorque = abs(PowTorque(PhiTol, Iref, 0.0))

def AllTorque(phi, omega, I1, I2):
    friqTorque = maxFrictionTorque*abs(omega)*0.1
    if friqTorque > maxFrictionTorque:
        friqTorque = maxFrictionTorque
    return PowTorque(phi, I1, I2) - friqTorque*np.sign(omega)

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
    print("t={0} phi={1} I1={2} I2={3}".format(t, y[0], y[2], y[3]))
    #return [y[1], AllTorque(y[0], y[1], y[2], y[3])/J, dI1dt(t, y[2]), dI2dt(t, y[3])]
    return [y[1], AllTorque(y[0], y[1], Iref1(t), Iref2(t))/J, 0.0, 0.0]

y0 = [0.0, 0.0, 0.0, 0.0]
sol = solve_ivp(MovementEquation, [0.0, time], y0, vectorized = True, dense_output = True)
t = np.linspace(0, time, 1000)
z = sol.sol(t)
plt.plot(t, z[0])
plt.xlabel("time")
plt.legend(["phi", "phi"], shadow = True)
plt.title("Stepper motor movement simulation")
plt.show()
