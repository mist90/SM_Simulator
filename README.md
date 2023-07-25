# SM_Simulator
Stepper motor simulator. Program simulates moving of hybrid stepper motor.
 - Program solves system of ODE for stepper motor:
$$\ \frac{d\varphi(t)}{dt} = \omega(t)$$
$$\ \frac{d\omega(t)}{dt} = (Et(t) - Dt(t) - Ft(t))/J$$
$$\ \frac{dI_{1}(t)}{dt} = \frac{U_{1}(t) - I_{1}(t)R}{L}$$
$$\ \frac{dI_{2}(t)}{dt} = \frac{U_{2}(t) - I_{2}(t)R}{L}$$
where:
$$\ M(t) = K(I_{1}(t)sin(N\varphi(t)) - I_{2}(t)cos(N\varphi(t)))$$ - electrical torque, Nm
$$\ Dt(t) = Dt_{max} sin(4N\varphi(t))$$ - detent torque of magnetic of stepper motor, Nm
$$\ Ft(t) = \frac{Ft_{max}\omega(t)}{|\omega(t)|}$$, but not more $\|Et(t) - Dt(t)|$ - friction torque, Nm
$$\ J$$ - moment of inertia, kg m^2
$$\ \varphi(t)$$ - mechanical angle of rotor, radian
$$\ \omega(t)$$ - angular velocity of rotor, rad/sec
$$\ I_{1}$$ - current of first phase of stepper motor, A
$$\ I_{2}$$ - current of second phase of stepper motor, A

 - Requires: python3, numpy, scipy
 - Run:
python3 SM_simulator.py
 - Parameters are inside SM_simulator.py file
 
 - StepMotor.asc - electromechanical LTSpice model of stepper motor. Example of using here:
 https://github.com/mist90/electrotherm
1. Clone that repository
2. Copy StepMotor.asc and StepMotor.asy files there
3. Open MotorControlStepping.asc
4. Run
