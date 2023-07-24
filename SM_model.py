import numpy as np
from scipy.integrate import solve_ivp

class StepperMotorSimulator:
    def __init__(self, params, driver):
        self.prevPercentage = 0
        self.driver = driver
        self.StopTime  =params["StopTime"]
        # Mechanical parameters of stepper motor
        self.N = params["N"]         # number of phases of stepper motor
        self.J = params["J"]         # Inertia moment, kg*m^2
        self.K = params["K"]         # Ratio of maximum torque to maximum phase current N*m/A
        self.PhiTol = params["PhiTol"] # tolerqnce, radian
        self.DT = params["DT"]       # Detent torque, N*m
        self.FT = params["FT"]       # Friction torque, N*m
        
        # Electrical parameters
        self.Vpow = params["Vpow"]   # Power voltage of phases of stepper motor
        self.L = params["L"]         # Phase inductance, Hn
        self.R = params["R"]         # Phase resistance, Ohm
    
    def GetVpow(self, t, iref, I):
        if iref > 0.0:
            if I < iref:
                return self.Vpow
            elif I - iref > 0.1:
                return -self.Vpow
            else:
                return I*self.R
        else:
            if I > iref:
                return -self.Vpow
            elif iref - I > 0.01:
                return self.Vpow
            else:
                return I*self.R
    
    def dIdt(self, t, iref, I):
        return (self.GetVpow(t, iref, I) - I*self.R)/self.L
    
    def ElectricTorque(self, phi, I1, I2):
        return self.K*(I1*np.sin(self.N*phi) - I2*np.cos(self.N*phi))
    
    def DetentTorque(self, phi):
        return self.DT*np.sin(self.N*phi*4.0)
    
    def SumTorque(self, phi, omega, I1, I2):
        powerTorque = self.ElectricTorque(phi, I1, I2) - self.DetentTorque(phi)
        friqTorque = self.FT
        if friqTorque > abs(powerTorque):
            friqTorque = abs(powerTorque)
        return powerTorque - friqTorque*np.sign(omega)
    
    """
    y has format:
    [phi(t), omega(t), I1(t), I2(t)],
    where:
    0. phi(t) - angle of rotor of stepper motor
    1. omega(t) - angle velocity  of rotor of stepper motor
    2. I1(t) - current of first phase
    3. I2(t) - current of second phase
    """
    def MovementEquation(self, t, y):
        percentage = int(t/self.StopTime*100.0)
        if percentage > self.prevPercentage:
            self.prevPercentage = percentage
            print("{0}%".format(percentage))
        return [y[1], self.SumTorque(y[0], y[1], y[2], y[3])/self.J, self.dIdt(t, self.driver.I1(t), y[2]), self.dIdt(t, self.driver.I2(t), y[3])]
    
    def run(self):
        y0 = [0.0, 0.0, 0.0, 0.0]
        self.sol = solve_ivp(self.MovementEquation, [0.0, self.StopTime], y0, vectorized = True, dense_output = True)
    
    def getPhi(self, t):
        return self.sol.sol(t)[0]
    
    def getOmega(self, t):
        return self.sol.sol(t)[1]
    
    def getI1(self, t):
        return self.sol.sol(t)[2]
    
    def getI2(self, t):
        return self.sol.sol(t)[3]

class FullStepDriver:
    def __init__(self, freqStart, freqStop, timeInterval, Iampl):
        self.freqStart = freqStart
        self.freqStop = freqStop
        self.timeInterval = timeInterval
        self.Iampl = Iampl
    def getCurFreq(self, t):
        return (self.freqStop - self.freqStart)/self.timeInterval*t + self.freqStart
    def I1(self, t):
        freq = self.getCurFreq(t)
        timeQuant = int(t*freq) % 4
        return self.Iampl if timeQuant < 2 else -self.Iampl
    def I2(self, t):
        freq = self.getCurFreq(t)
        timeQuant = int(t*freq) % 4
        return self.Iampl if timeQuant > 0 and timeQuant < 3 else -self.Iampl

class MicroStepDriver:
    def __init__(self, freqStart, freqStop, timeInterval, Iampl):
        self.freqStart = freqStart
        self.freqStop = freqStop
        self.timeInterval = timeInterval
        self.Iampl = Iampl
    def getCurFreq(self, t):
        return (self.freqStop - self.freqStart)/self.timeInterval*t + self.freqStart
    def I1(self, t):
        freq = self.getCurFreq(t)
        return self.Iampl*np.cos(2.0*np.pi*freq*t)
    def I2(self, t):
        freq = self.getCurFreq(t)
        return self.Iampl*np.sin(2.0*np.pi*freq*t)