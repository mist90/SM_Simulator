# SM_Simulator
Stepper motor simulator. Program simulates moving of hybrid stepper motor.
 - Program solves system of ODE for stepper motor:
$$\ \frac{d\varphi(t)}{dt} = \omega(t)$$
$$\ \frac{d\omega(t)}{dt} = (Et(t) - Dt(t) - Ft(t))/J$$
$$\ \frac{dI_{1}(t)}{dt} = \frac{U_{1}(t) - I_{1}(t)*R}{L}$$
$$\ \frac{dI_{2}(t)}{dt} = \frac{U_{2}(t) - I_{2}(t)*R}{L}$$
where:
$$\ M(t) = K(I_{1}(t)sin(N\varphi(t)) - I_{2}(t)cos(N\varphi(t)))$$ - electrical torque
$$\ Dt(t) = Dt_{max} sin(4N\varphi(t))$$ - detent torque of magnetic of stepper motor
$$\ Ft(t) = \frac{Ft_{max}\omega(t)}{|\omega(t)|}$$, but not more $\|Et(t) - Dt(t)|$ - friction torque

 - Requires: python3, numpy, scipy
 - Run:
puthon3 SM_simulator.py
 - Parameters inside SM_simulator.py file
