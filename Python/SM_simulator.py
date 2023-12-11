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
import sys
import configparser

params = dict()

for i in range(1, len(sys.argv)):
    cfg = configparser.ConfigParser()
    cfg.read(sys.argv[i])
    for sectionName in cfg:
        section = dict(cfg[sectionName])
        if sectionName == "Mechanical":
            params["N"] = int(section["n"])           # number of phases of stepper motor
            params["J"] = float(section["j"])         # Inertia moment, kg*m^2
            params["K"] = float(section["k"])         # Ratio of maximum torque to maximum phase current N*m/A
            params["FTc"] = float(section["ftc"])     # Coulomb Friction torque, N*m
            params["FTbrk"] = float(section["ftbrk"]) # Breakaway Friction torque, N*m
            params["brkOmega"] = float(section["brkomega"])   # Breakaway angular velocity, 1/s
            params["B"] = float(section["b"])         # Viscous damping friction torque constant, N*m*s
        elif sectionName == "Electrical":
            params["L"] = float(section["l"])         # Phase inductance, Hn
            params["R"] = float(section["r"])         # Phase resistance, Ohm
            params["Flowmax"] = float(section["flowmax"]) # Maximum magnetic flow of stepper motor magnetics, Wb
            params["DT"] = float(section["dt"])       # Detent torque, N*m
        elif sectionName == "Time":
            params["StopTime"] = float(section["stoptime"])
            numPoints = int(section["numpoints"])
        elif sectionName == "Frequency":
            startFreq = float(section["startfreq"])
            stopFreq = float(section["stopfreq"])
            params["startOmega"] = float(section["startomega"]) # 2.0*np.pi*startFreq/params["N"]
        elif sectionName == "Output":
            Iref = float(section["iref"])      # Phase current, A
            params["Vpow"] = float(section["vpow"])     # Power voltage of phases of stepper motor
            microstep = True if section["microstep"] == "true" else False
            """True - set voltage reverse polarity for decreasing current in motor phase,
               False - set 0 V for decreasing current in motor phase"""
            params["breakByVoltage"] = True if section["breakbyvoltage"] == "true" else False

time = params["StopTime"]

# Select driver
if microstep:
    driver = SM_model.MicroStepDriver(startFreq, stopFreq, time, Iref)
else:
    driver = SM_model.FullStepDriver(startFreq, stopFreq, time, Iref)

sm = SM_model.StepperMotorSimulator(params, driver)
sm.run()
t = np.linspace(0, time, numPoints)

# Plot results
fig, axs = plt.subplots(3, 1, sharex=True)
# Remove vertical space between axes
fig.subplots_adjust(hspace=0)

# Plot each graph
axs[0].plot(t, sm.getPhi(t))
axs[0].set_ylabel("phi, rad")
axs[1].plot(t, sm.getOmega(t))
axs[1].set_ylabel("w, rad/s")
axs[2].plot(t, sm.getI1(t))
axs[2].plot(t, sm.getI2(t))
axs[2].set_ylabel("I1,I2, A")

plt.xlabel("time")
fig.suptitle("Stepper motor movement simulation")
plt.show()
