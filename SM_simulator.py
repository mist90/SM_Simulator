#    MIT License
#  Copyright (c) 2023 Mikhail Tegin
#  michail3110@gmail.com
#  
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
# 
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
# 
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.

import numpy as np
import matplotlib.pyplot as plt
import SM_model

# Parameters of simulator
startFreq = 25.0 # frequency of stepper motor are changed linearly from startFreq to stopFreq, 1/s
stopFreq = 25.1 # frequency of stepper motor are changed linearly from startFreq to stopFreq, 1/s
time = 0.32     # seconds
numPoints = 200000
Iref = 2.8      # Phase current, A

params = dict()
params["StopTime"] = time
# Mechanical parameters of stepper motor
params["N"] = 200         # number of phases of stepper motor
params["J"] = 570E-07     # Inertia moment, kg*m^2
params["K"] = 21.0E-02*9.8/2.8    # Ratio of maximum torque to maximum phase current N*m/A
params["FTc"] = 0.051      # Coulomb Friction torque, N*m
params["FTbrk"] = 0.051    # Breakaway Friction torque, N*m
params["brkOmega"] = 0.1   # Breakaway angular velocity, 1/s
params["B"] = 0.0          # Viscous damping friction torque constant, N*m*s
params["startOmega"] = 0.0 # 2.0*np.pi*startFreq/params["N"]

# Electrical parameters
params["Iref"] = 2.8      # Phase current, A
params["Vpow"] = 24.0     # Power voltage of phases of stepper motor
params["L"] = 4.6E-03     # Phase inductance, Hn
params["R"] = 1.2         # Phase resistance, Ohm

# Calculation of DT parameter
k = params["K"]/params["L"]   # T = K*I = k*F = k*L*I
Vmax = 62.6               # Max voltage of induced field, V
fmax = 905.44             # Frequency of induced field at Vmax, Hz
Fmax = Vmax/(2.0*np.pi*fmax) # Maximum magnetic flow of stepper motor magnetics, Wb
params["DT"] = k*Fmax     # Detent torque, N*m


driver = SM_model.FullStepDriver(startFreq, stopFreq, time, Iref)
#driver = SM_model.MicroStepDriver(startFreq, stopFreq, time, Iref)
sm = SM_model.StepperMotorSimulator(params, driver)
sm.run()
t = np.linspace(0, time, numPoints)

# Plot results
fig, axs = plt.subplots(4, 1, sharex=True)
# Remove vertical space between axes
fig.subplots_adjust(hspace=0)

# Plot each graph
axs[0].plot(t, sm.getPhi(t))
axs[0].set_ylabel("phi, rad")
axs[1].plot(t, sm.getOmega(t))
axs[1].set_ylabel("w, rad/s")
axs[2].plot(t, sm.getI1(t))
axs[2].set_ylabel("I1, A")
axs[3].plot(t, sm.getI2(t))
axs[3].set_ylabel("I2, A")

plt.xlabel("time")
plt.title("Stepper motor movement simulation")
plt.show()
