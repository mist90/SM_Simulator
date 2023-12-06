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
from scipy.integrate import solve_ivp

class Switcher:
    def __init__(self, PWMfreq, Vpow):
        self.PWMfreq = PWMfreq
        self.Vpow = Vpow
        self.lastDownQuant = 0
        self.downMode = False
    
    def GetVoltage(self, t, iref, I):
        if iref >= 0.0:
            if I > iref:
                if not self.downMode:
                    self.downMode = True
                    self.lastTimeDown = int(t*self.PWMfreq)
            else:
                numCurQuant = int(t*self.PWMfreq)
                if numCurQuant > self.lastDownQuant:
                    self.downMode = False
            return self.Vpow if not self.downMode else 0.0
        else:
            if I < iref:
                if not self.downMode:
                    self.downMode = True
                    self.lastTimeDown = int(t*self.PWMfreq)
            else:
                numCurQuant = int(t*self.PWMfreq)
                if numCurQuant > self.lastDownQuant:
                    self.downMode = False
            return -self.Vpow if not self.downMode else 0.0

class StepperMotorSimulator:
    def __init__(self, params, driver):
        self.prevPercentage = 0
        self.driver = driver
        self.StopTime  =params["StopTime"]
        self.PWMfreq = 30000.0
        # Mechanical parameters of stepper motor
        self.N = params["N"]         # number of phases of stepper motor
        self.J = params["J"]         # Inertia moment, kg*m^2
        self.K = params["K"]         # Ratio of maximum torque to maximum phase current N*m/A
        self.DT = params["DT"]       # Detent torque, N*m
        self.FTc = params["FTc"]     # Coulomb Friction torque, N*m
        self.FTbrk = params["FTbrk"] # Breakaway Friction torque, N*m
        self.brkOmega = params["brkOmega"] # Breakaway angular velocity, 1/s
        self.B = params["B"]         # Viscous damping friction torque constant, N*m*s
        self.startOmega = params["startOmega"]
        
        # Electrical parameters
        self.Vpow = params["Vpow"]   # Power voltage of phases of stepper motor
        self.L = params["L"]         # Phase inductance, Hn
        self.R = params["R"]         # Phase resistance, Ohm
        self.Flowmax = params["Flowmax"]   # Max magnetic Flow
        
        self.sw1 = Switcher(self.PWMfreq, self.Vpow)
        self.sw2 = Switcher(self.PWMfreq, self.Vpow)
    
    def GetVpow(self, t, iref, I, switcher):
        return switcher.GetVoltage(t, iref, I)
    
    def GetInducedVoltage1(self, phi, omega):
        return -self.N/2.0*self.Flowmax*omega*np.sin(self.N/2.0*phi)
    
    def GetInducedVoltage2(self, phi, omega):
        return self.N/2.0*self.Flowmax*omega*np.cos(self.N/2.0*phi)
    
    def dIdt1(self, t, phi, omega, I):
        return (self.GetVpow(t, self.driver.I1(t), I, self.sw1) - self.GetInducedVoltage1(phi, omega) - I*self.R)/self.L
    
    def dIdt2(self, t, phi, omega, I):
        return (self.GetVpow(t, self.driver.I2(t), I, self.sw2) - self.GetInducedVoltage2(phi, omega) - I*self.R)/self.L
    
    def ElectricTorque(self, phi, I1, I2):
        return -self.K*(I1*np.sin(self.N/2.0*phi) - I2*np.cos(self.N/2.0*phi))
    
    def DetentTorque(self, phi):
        return -self.DT/10.0*np.sin(4.0*self.N/2.0*phi)
    
    def FrictionTorque(self, omega):
        wst = self.brkOmega*np.sqrt(2.0)
        wcoul = self.brkOmega/10.0
        return np.sqrt(2.0*np.e)*(self.FTbrk - self.FTc)*np.exp(-(omega/wst)**2)*omega/wst + self.FTc*np.tanh(omega/wcoul) + self.B*omega
    
    def SumTorque(self, phi, omega, I1, I2):
        return self.ElectricTorque(phi, I1, I2) + self.DetentTorque(phi) - self.FrictionTorque(omega)
    
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
        return [y[1], self.SumTorque(y[0], y[1], y[2], y[3])/self.J, self.dIdt1(t, y[0], y[1], y[2]), self.dIdt2(t, y[0], y[1], y[3])]
    
    def run(self):
        y0 = [0.0, self.startOmega, 0.0, 0.0]
        self.sol = solve_ivp(self.MovementEquation, [0.0, self.StopTime], y0, vectorized = True, dense_output = True, max_step = 0.25/self.PWMfreq)
    
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
