# Self Balancing Robot

[![Watch the video](/IMAGE/CAD.png)](https://drive.google.com/file/d/1cC-Chdwye79UuMQegcqwbnqvilcww2Fp/view?usp=drive_link)

This is the project involving the development of a Self Balancing Robot. It uses the Proportional, Integraal and Derivative(PID) control algorithm to balance itself. It uses the 6 Degrees of Freedom(DOF) Inertial Measurement Unit(IMU) as the sensor to get the angle data.

<br>
**Process Variable:**  Angle(Roll) (deg)
<br>
**Control Variable:**  Speed of the Motors (mm/sec)
<br>
**Setpoint        :**  0 deg (calibrated for upright position)

<br>

The motors used are the stepper motors hence the relationship between the number of steps and speed is non-linear in nature, hence PID algorithm can not be used unless the linearization is performed.

<br>
$$ Speed = {157 \over 6400*(40+20n)*10^(-6)} $$
Here the speed is calculated in mm/sec, and **n** is the number of steps.
<br>
$$ n = {(157*10^(6) /over 5400*Speed*20) - (40 /over 20)} $$
<br>

To mitigate the problem of nonlinear-system, the PID algoritm is implemented on **Speed** rather than the number of steps **n**.
