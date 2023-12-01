# SM_Simulator
Stepper motor simulator. Program simulates moving of hybrid stepper motor.
 - Program solves system of ODE for stepper motor:
$$\ \frac{d\varphi(t)}{dt} = \omega(t)$$
$$\ \frac{d\omega(t)}{dt} = (T_{e}(t) - T_{d}(t) - T_{f}(t))/J$$
$$\ \frac{dI_{1}(t)}{dt} = \frac{V_{1}(t) + V_{emf1}(t) - I_{1}(t)R}{L}$$
$$\ \frac{dI_{2}(t)}{dt} = \frac{V_{2}(t) + V_{emf2}(t) - I_{2}(t)R}{L}$$
where:
$$\ N $$ - number of poles of stepper motor
$$\ T_{e}(t) = K(I_{1}(t)sin(N/2\varphi(t)) - I_{2}(t)cos(N/2\varphi(t)))$$ - electrical torque, Nm
$$\ T_{d}(t) = T_{d max} sin(4*N/2\varphi(t))$$ - detent torque of magnetic of stepper motor, Nm
$$\ T_{f}(t) = \sqrt{2e}(T_{brk} - T_{c}){e}^{-(\frac{\omega(t)}{\omega_{brk} \sqrt{2}})^{2}}\frac{\omega(t)}{\omega_{brk} \sqrt{2}} + T_{c} tanh(\frac{\omega(t)}{\omega_{brk}/10})$$ - friction torque, Nm
$$\ J$$ - moment of inertia, kg m^2
$$\ V_{1}(t)$$ - voltage on first phase of stepper motor, V
$$\ V_{2}(t)$$ - voltage on second phase of stepper motor, V
$$\ V_{emf1}(t) = N/2 F_{max}\omega(t)cos(N/2\varphi(t))$$ - back EMF voltage of first phase of stepper motor, V
$$\ V_{emf2}(t) = -N/2 F_{max}\omega(t)sin(N/2\varphi(t))$$ - back EMF voltage of second phase of stepper motor, V
$$\ F_{max} = \frac{T_{d} L}{K} $$ - maximum magnetic flow of poles
$$\ \varphi(t)$$ - mechanical angle of rotor, radian
$$\ \omega(t)$$ - angular velocity of rotor, rad/sec
$$\ I_{1}$$ - current of first phase of stepper motor, A
$$\ I_{2}$$ - current of second phase of stepper motor, A

 - Requires: python3, numpy, scipy
 - Run:
python3 SM_simulator.py
 - Parameters are inside SM_simulator.py file
 
 - LTSpice/StepMotor.asc - electromechanical LTSpice model of stepper motor. Example of using in MotorControlStepping.asc:
1. Open MotorControlStepping.asc
2. Run
