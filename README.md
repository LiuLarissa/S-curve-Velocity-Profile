# S-Curve-Velocity-Profile-2.5-Axis-Milling
This program generates a velocity profile for milling CNC in light of S-curve calculations. It is available to process linear and circular phases. There are many papers and books about the generation of S-curve velocity profiles but I couldn't find a detailed solution. Therefore, I code this program with references to these papers and books. 

## Introduction of S-Curve
S-curve (aka. jerk-limited trajectory generation algorithm) is used to smooth velocity and acceleration profiles by considering the jerk. The kinematic time profiles of jerk (J), acceleration (a), feed rate ($f$), and trajectory command position ($l$) are shown in Fig.1.

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/Jerk-limited-algorithm.png)

Figure.1 Kinematic profile for jerk-limited feed rate generation

Before the NC block motion is started, the initial and final values of position $(l_s,l_e)$ and feedrate $(f_s,f_e)$, maximum acceleration (A), deceleration (D), and jerk (J) limits are defined. The maximum acceleration/deceleration limits are identified from the maximum torque and force limits of the drive motors. The acceleration time is set depending on the peak torque/force delivery periods of the amplifier. The jerk limit is set by the maximum acceleration divided by the acceleration time. A whole S-curve contains seven phases. The jerk, acceleration, feed, and displacement along the tool path are expressed as follows:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/jerk.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acceleration.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/feed.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/displacement.png)

where $l_k$ is the total displacement reached at the end of phase k. The incremental distance $L_k$ traveled at each phase (k) is

$$
L_k=l_k-l_{k-1}
$$


where the initial displacement is $l_0 = l_s$. 

$A=J_1T_1=J_3T_3$, $D=J_5T_5=J_7T_7$, should hold, although this may require readjustment after the first initialization step. Considering that the desired feed ($f$) is reached at the end of phase 3, we have

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/desired-feed.png)

and similarly, considering that the final feed ($f_e$) is reached at the end of phase 7, we have 

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/final-feed.png)

The total number of interpolation steps (N) is checked first. If 2<N<=4, then N=4 is selected to guarantee at least the presence of the acceleration and deceleration phases (1,3,5, and 7) in Fig.1. If N<=2, then N=2 is selected to allow acceleration and deceleration. These conditions would only occur when the motion is very small, such as in high-speed spline interpolation applications or precision positioning. If the acceleration stage exists, the desired feed ($f$) must be reached within the first three phases, which implies that $T_2>=0$. If the jerk values are equal ($J_1=J_3$), the acceleration condition requires that

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acc-condition.png)

If this equation does not hold, then the magnitude of the acceleration must be reduced to its maximum possible limit as

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/maximum-acc.png)

and $T_2$ is set to zero. The deceleration phase is similar to the acceleration phase. If the displacement of the acceleration phase and the deceleration phase is large enough to encompass the constant feed stage, $T_4>=0$ must be identified.

The above introduction is referenced from "Manufacturing Automation" written by Yusuf Altintas.

## Simplification of Equations

In my opinion, the above equations can be simplified as follows:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/simplified_equation.png)

where L<sub>a</sub> and L<sub>d</sub> are calculated as follows:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acc_dec_displacement.png)

and:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/conditions.png)


Here, $V_s$ , $V_e$ , A, J, $F_{max}$ , and L are known, while $T_1$, $T_2$, $T_4$, $T_5$, $T_6$ need to be solved. It is a non-linear and nonhomogeneous equation. Therefore, I decided to use scipy.optimize.minimize to solve this equation. Figure.2 is the flow chart.

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/VelocityScheduleFlowChart1.png)

Figure.2 Velocity schedule flow chart

First, A, J, $V_s$, $V_e$, $F_{max}$ are used to calculate $T_1$, $T_2$, $T_5$, $T_6$. If $T_2$ is less than 0, set $T_2=0$ and re-calculate $T_1$. Then, $T_1$ and $T_2$ are used to calculate $L_a$. The calculation of $T_5$, $T_6$, and $L_d$ is the same with using $V_e$ instead of $V_s$.

After that, if $L_a+L_d<=L$, the feedrate-constant phase exists. $T_4$ is calculated. Otherwise, $T_4$ equals to 0.

Then, in order to solve $T_1$, $T_2$, $T_5$, $T_6$, I transform this non-linear and nonhomogeneous equation into a minimization problem. That is, find the optimal $T_1$, $T_2$, $T_5$, $T_6$ to minimize the value of the following function:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/TargetFunction.png)

considering the following boundaries and constraints:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/boundaries.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/Constraints.png)

The cases of $T_2$ and $T_6$ equal to 0 are discussed independently. 

## Calculate $V_s$ and $V_e$

$V_e$ of a block is determined by the direction of its successive block, as shown in Fig.3. The feedrate of the first block is $F_1$, the feedrate of the second block is $F_2$. The angle between them is $\theta$. 

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/difference-direction-feedrate.png)

Figure.3 Two successive blocks with different directions

The acceleration at the corner is computed by:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acc-corner.png)

where, $T_{pos}$ is the sampling time for position control. Therefore, a corner speed $F_c$ is calcuated by:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/feedrate-corner.png)

The above content is referenced from Chapter 4 in "Theory and design of CNC".

Moreover, $V_e$ is influenced by the chord error, as shown in Fig.4.

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/chorderror.png)

Figure.4 Chord Error

## Discretization of time

$T_1$, $T_2$, $T_4$, $T_5$, $T_6$ should be an integral multiple of the sampling time for position control $T_s$ (or interpolation period). Therefore, in this program:

$$
T_1=floor(\frac{T_1}{T_s})*T_s 
$$

Then, Jerka, Jerkd, Acc, Dec should be re-calculated.

Figure.5 presents an example. The G code program and its interpreted target program have been uploaded.


![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/example.png)




