import numpy as np
import matplotlib.pyplot as plt
import SM_model

# Parameters of simulator
maxFreq = 200.0 # frequency of stepper motor are changed linearly from 0 to maxFreq, 1/s
time = 20.0     # seconds
numPoints = 200000
Iref = 2.8      # Phase current, A

params = dict()
params["StopTime"] = time
# Mechanical parameters of stepper motor
params["N"] = 200         # number of phases of stepper motor
params["J"] = 570E-07     # Inertia moment, kg*m^2
params["K"] = 31.0E-02*9.8/2.8    # Ratio of maximum torque to maximum phase current N*m/A
params["DT"] = 0.09       # Detent torque, N*m
params["FT"] = 0.04       # Friction torque, N*m

# Electrical parameters
params["Iref"] = 2.8      # Phase current, A
params["Vpow"] = 24.0     # Power voltage of phases of stepper motor
params["L"] = 6.8E-03     # Phase inductance, Hn
params["R"] = 1.5         # Phase resistance, Ohm


driver = SM_model.FullStepDriver(1.0, maxFreq, time, Iref)
#driver = SM_model.MicroStepDriver(1.0, maxFreq, time, Iref)
sm = SM_model.StepperMotorSimulator(params, driver)
sm.run()
t = np.linspace(0, time, numPoints)
plt.plot(t, sm.getPhi(t))
plt.xlabel("time")
plt.legend(["phi"], shadow = True)
plt.title("Stepper motor movement simulation")
plt.show()
