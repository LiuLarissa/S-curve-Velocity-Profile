# S-Curve-Velocity-Profile-2.5-Axis-Milling
This program generates velocity profile for milling CNC in light of S-curve calculations. It is available to process linear and circular phases. There are many papers and books about the generation of S-curve velocity profiles but I couldn't find a detailed solution. Therefore, I code this program with referenced these papers and books. 

## Introduction of S-Curve
S-curve (aka. jerk-limited trajectory generation algorithm) is used to smooth velocity and acceleration profiles by considering the jerk. The kinematic time profiles of jerk (J), accleration (a), feed rate (f), and trajectory command position (l) are shown in Fig.1.

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/Jerk-limited-algorithm.png)

Figure.1 Kinematic profiles for jerk-limited feed rate generation

Before the NC block motion is started, the initial and final values of position (ls, le) and feed rate (fs, fe), maximum acceleration (A), deceleration (D), and jerk (J) limits are defined. The maximum acceleration/deceleration limits are identified from the maximum torque and force limits of the drive motors. The acceleration time is set depending on the peak torque/force delivery periods of the amplifier. The jerk limit is set by the maximum acceleration divided by the acceleration time. A whole S-curve contains seven phases. The jerk, acceleration, feed, and displacement along the tool path are expressed as follows:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/jerk.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acceleration.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/feed.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/displacement.png)

where l<sub>k</sub> is the total displacement reached at the end of phase k. The incremental distance L<sub>k</sub> traveled at each phase (k) is

L<sub>k</sub> = l<sub>k</sub> - l<sub>k-1</sub>

where the initial displacement is l<sub>0</sub> = l<sub>s</sub>. 

A=J<sub>1</sub>T<sub>1</sub>=J<sub>3</sub>T<sub>3</sub>, D=J<sub>5</sub>T<sub>5</sub>=J<sub>7</sub>T<sub>7</sub>, should hold, although this may require readjustment after the first initialization step. Considering that the desired feed (f) is reached at the end of phase 3, we have

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/desired-feed.png)

and similarly, considering that the final feed (f<sub>e</sub>) is reached at the end of phase 7, we have 

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/final-feed.png)

The total number of interpolation steps (N) is checked firstly. If 2<N<=4, then N=4 is selected to guarantee at least the presence of the acceleration and deceleration phases (1,3,5, and 7) in Fig.1. If N<=2, then N=2 is selected to allow an acceleration and deceleration. These conditions would only occur when the motion is very small, such as in high-speed spline interpolation applications or in precision positioning. If the acceleration stage exists, the desired feed (f) must be reached within the first three phases, which implies that T<sub>2</sub>>=0. If the jerk values are equal (J<sub>1</sub>=J<sub>3</sub>), the acceleration condition requires that

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acc-condition.png)

If this equation does not hold, then the magnitude of the acceleration must be reduced to its maximum possible limit as

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/maximum-acc.png)

and T<sub>2</sub> is set to zero. The deceleration phase is similar to the acceleration phase. If the displacement of the acceleration phase and the deceleration phase is large enough to encompass the constant feed stage, T<sub>4</sub>>=0 and must be identified.

The above introduction is referenced from "Manufacturing Automation" written by Yusuf Altintas.

## Simplification of Equations

In my opinion, the above equations can be siplified as follows:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/simplified_equation.png)

where L<sub>a</sub> and L<sub>d</sub> are calculated as follows:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acc_dec_displacement.png)

and:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/conditions.png)


Here, V<sub>s</sub>, V<sub>e</sub>, A, J, F<sub>max</sub>, L are defined, T<sub>1</sub>, T<sub>2</sub>, T<sub>4</sub>, T<sub>5</sub>, T<sub>6</sub> need to be solved. This is a non-linear and nonhomogeneous equation. Therefore, I decided to use scipy.optimize.minimize to solve this equation. Figure.2 is the flow chart.

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/VelocityScheduleFlowChart.png)

Figure.2 Velocity schedule flow chart

First, A, J, V<sub>s</sub>, V<sub>e</sub>, F<sub>max</sub> are used to calculate T<sub>1</sub>, T<sub>2</sub>, T<sub>5</sub>, T<sub>6</sub>. If T<sub>2</sub> is less than 0, set T<sub>2</sub>=0 and re-calculate T<sub>1</sub>. Then, T<sub>1</sub> and T<sub>2</sub> are used to calculate L<sub>a</sub>. The calculation of T<sub>5</sub>, T<sub>6</sub>, and L<sub>d</sub> is the same with using V<sub>e</sub> instead of V<sub>s</sub>.

After that, if L<sub>a</sub> plus L<sub>d</sub> is not more than L, the feedrate-constant phase exists. T<sub>4</sub> is calculated. Otherwise, T<sub>4</sub> equals to 0.

Then, in order to solve T<sub>1</sub>, T<sub>2</sub>, T<sub>5</sub>, T<sub>6</sub>, I transform this non-linear and nonhomogeneous equation into a minimization problem. That is, find the optimal T<sub>1</sub>, T<sub>2</sub>, T<sub>5</sub>, T<sub>6</sub> to minimize the value of the following function:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/TargetFunction.png)

considering the following boundaries and constraints:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/boundaries.png)

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/Constraints.png)

The cases of T<sub>2</sub> and T<sub>6</sub> equal to 0 are discussed independently. 

## Calculate V<sub>s</sub> and V<sub>e</sub>

V<sub>e</sub> of a block is determined by the direction of its successive block, as shown in Fig.3. The feedrate of the first block is F<sub>1</sub>, the feedrate of the second block is F<sub>2</sub>. The angle between them is \theta. 

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/difference-direction-feedrate.png)

Figure.3 Two successive blocks with different directions

The acceleration at the corner is computed by:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/acc-corner.png)

where, T<sub>pos</sub> is the sampling time for position control. Therefore, a corner speed F<sub>c</sub> is calcuated by:

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/feedrate-corner.png)

The above content is referenced from Chapter 4 in "Theory and design of CNC".

Moreover, V<sub>e</sub> is influenced by the chord error, as shown in Fig.4.

![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/chorderror.png)

Figure.4 Chord Error

## Discretization of time

T<sub>1</sub>, T<sub>2</sub>, T<sub>4</sub>, T<sub>5</sub>, T<sub>6</sub> should be an integral multiple of the sampling time for position control T<sub>s</sub> (or interpolation period). Therefore, in this program:

T<sub>1</sub> = floor(T<sub>1</sub>/T<sub>s</sub>) * T<sub>s</sub>

Then, Jerka, Jerkd, Acc, Dec should be re-calculated.

Figure.5 presents an example. The G code program and its interpreted target program have been uploaded.


![image](https://github.com/Larissa1990/S-curve-Velocity-Profile/blob/master/images/example.png)




