# SM_Simulator
Stepper motor simulator. Program simulates moving of hybrid stepper motor.
 - Program solves system of ODE for stepper motor:
$$\ \frac{d\varphi(t)}{dt} = \omega(t)$$
$$\ \frac{d\omega(t)}{dt} = (T_{e}(t) + T_{d}(t) - T_{f}(t))/J$$
$$\ \frac{dI_{1}(t)}{dt} = \frac{V_{1}(t) + V_{emf1}(t) - I_{1}(t)R}{L}$$
$$\ \frac{dI_{2}(t)}{dt} = \frac{V_{2}(t) + V_{emf2}(t) - I_{2}(t)R}{L}$$
where:
$$\ N $$ - number of poles of stepper motor
$$\ T_{e}(t) = -K(I_{1}(t)sin(N/2\varphi(t)) - I_{2}(t)cos(N/2\varphi(t)))$$ - electrical torque, Nm
$$\ T_{d}(t) = -T_{d max} sin(4*N/2\varphi(t))$$ - detent torque of magnetic of stepper motor, Nm
$$\ T_{f}(t) = \sqrt{2e}(T_{brk} - T_{c}){e}^{-(\frac{\omega(t)}{\omega_{brk} \sqrt{2}})^{2}}\frac{\omega(t)}{\omega_{brk} \sqrt{2}} + T_{c} tanh(\frac{\omega(t)}{\omega_{brk}/10})$$ - friction torque, Nm
$$\ J$$ - moment of inertia, kg m^2
$$\ V_{1}(t)$$ - voltage on first phase of stepper motor, V
$$\ V_{2}(t)$$ - voltage on second phase of stepper motor, V
$$\ V_{emf1}(t) = -N/2 F_{max}\omega(t)sin(N/2\varphi(t))$$ - back EMF voltage of first phase of stepper motor, V
$$\ V_{emf2}(t) = N/2 F_{max}\omega(t)cos(N/2\varphi(t))$$ - back EMF voltage of second phase of stepper motor, V
$$\ F_{max} = \frac{T_{d} L}{K} $$ - maximum magnetic flow of poles
$$\ \varphi(t)$$ - mechanical angle of rotor, radian
$$\ \omega(t)$$ - angular velocity of rotor, rad/sec
$$\ I_{1}$$ - current of first phase of stepper motor, A
$$\ I_{2}$$ - current of second phase of stepper motor, A

 - Scheme of directions of vectors:
![scheme](https://github.com/mist90/SM_Simulator/assets/99616450/a466799b-4758-48aa-b135-c77267a4cb84)

1. Python model
 - Requires: python3, numpy, scipy, matplotlib
 - Run Python model:
cd ./Pyhton
python3 SM_simulator.py
 - Parameters are inside SM_simulator.py file

2. LTSpice model
Schemes:
* MotorControlStepping.asc - movement simulation of stepper motor with electro-thermal scheme of stepper motor driver
* MotorControlSteppingSimplfied.asc - simplified scheme of movement simulation of stepper motor without electro-thermal scheme of stepper motor driver
* MotorControlFreeRotation.asc - free rotation simulation of stepper motor until stopping by friction force

Subcircuits:
* StepMotor.asc - electro-mechanical scheme of stepper motor.
* HalfBridge.asc - scheme of half bridge for current stabilization and control in step motor.
	Dependent of subcircuit: IRFB4615PBF_therm.asc, feedback.asc
* Radiator.asc - thermal scheme of heat sink Wakefield OMNI-UNI-30-50-D
* IRFB4615PBF_therm.asc - electro-thermal model of transistor IRFB4615PBF
	Dependent of subcircuit: irfb4615pbf.asy
* feedback.asc - logical block of current stabilization
* irfb4615pbf.asy - symbol for spice model irfb4615pbf.spi

Quick start:
cd ./LTSpice
 - Download Infineon spice-model file of transistor irfb4615pbf.spi.
 Link: https://www.infineon.com/dgdl/irfb4615pbf.spi?fileId=5546d462533600a4015357128143396c
 - Change next parameters for .MODEL MM:
.MODEL MM NMOS LEVEL=3 IS=1e-32
+VTO=Vth NFS=1.988288E+12 KP=74.89621574661534 THETA=0.31480607500134106 KAPPA=42.67943561393658 VMAX = 4E+3
+NSUB=1E+15 PHI=0.576 GAMMA=0.5276 TOX=1E-07 XJ = 0 DELTA=0
 - Change RS:
RS 8 3 0.0001
 - Change RD:
RD 9 1 {Rds_val}
 - Open asc-file in LTSpice and press Run button
 - Run
