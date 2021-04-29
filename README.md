# S-Curve-Velocity-Profile-Milling
This program generates velocity profile for milling CNC in light of S-curve calculations.

## Introduction of S-Curve
S-curve (aka. jerk-limited trajectory generation algorithm) is used to smooth velocity and acceleration profiles by considering the jerk. The kinematic time profiles of jerk (J), accleration (a), feed rate (f), and trajectory command position (l) are shown in Fig.1.

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/Jerk-limited-algorithm.png)

Fig.1 Kinematic profiles for jerk-limited feed rate generation

Before the NC block motion is started, the initial and final values of position (ls, le) and feed rate (fs, fe), maximum acceleration (A), deceleration (D), and jerk (J) limits are defined. The maximum acceleration/deceleration limits are identified from the maximum torque and force limits of the drive motors. The acceleration time is set depending on the peak torque/force delivery periods of the amplifier. The jerk limit is set by the maximum acceleration divided by the acceleration time. The jerk, acceleration, feed, and displacement along the tool path are expressed as follows:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/jerk.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acceleration.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/feed.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/displacement.png)

where l(k) is the total displacement reached at the end of phase k. The incremental distance L(k) traveled at each phase (k) is

L(k) = l(k) - l(k-1)

where the initial displacement is l(0) = l(s). 

A=J1T1=J3T3, D=J5T5=J7T7, should hold, although this may require readjustment after the first initialization step. Considering that the desired feed (f) is reached at the end of phase 3, we have

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/desired feed.png)

and similarly, considering that the final feed (fe) is reached at the end of phase 7, we have 

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/final feed.png)









