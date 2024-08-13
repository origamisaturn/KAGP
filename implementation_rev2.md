## Table of Contents

- [1. Guidance Objects](#1-guidance-objects)
    - [1.1. OrbitTargetingAscent](#11-orbittargetingascent)
    - [1.2. DebugAscent1](#12-debugascent1)
- [2. Simulation Objects](#2-simulation-objects)
- [Appendix A: Symbols](#appendix-a-symbols)
- [Appendix B: Reference Frames](#appendix-b-reference-frames)
- [Appendix C: Abbreviated Derivation](#appendix-c-abbreviated-derivation)
    - [C.1. Fixed-Thrust Model](#c1-fixed-thrust-model)
    - [C.2. Generalized Guidance Law](#c2-generalized-guidance-law)
    - [C.3. Radial Guidance Law](#c3-radial-guidance-law)
    - [C.4. Plane Control Guidance Law](#c4-plane-control-guidance-law)
    - [C.5. Time-to-Go](#c5-time-to-go)
    - [C.6. Final Circumferential Velocity](#c6-final-circumferential-velocity)
    - [C.7. Pitch and Heading](#c7-pitch-and-heading)
    - [C.8. Final True Anomaly](#c8-final-true-anomaly)
    - [C.9. Orbit Targeting](#c9-orbit-targeting)
- [References](#references)

## 1. Guidance Objects

### 1.1. OrbitTargetingAscent

### 1.2. DebugAscent1

## 2. Simulation Objects

### 2.1. IntegratorSim 

### 2.2. KRPCClient

## 3. Guidance Components

### 3.1. RadialYawGuidance

### 3.1. TimeToGo

### 3.2. VThetaSolver

### 3.3. PitchHeadingQuery

### 3.4. OrbitTargeting

### 3.5. EnginePropertyEstimator

## Appendix A: Symbols

| Symbol | Description |
| ---   | ---   |
| $A$, $B$ | steering constants |
| $a_0$, $a_1$, ... $a_n$ | thrust acceleration coefficients |
| $a_T$ | thrust acceleration, $m/s^2$ |
| $\alpha$ | pitch angle, $rad.$ |
| $\alpha_y$ | yaw angle, $rad.$ |
| $c_1$, $c_2$ | steering constants, <br> [nondimensional, $s^{-1}$] |
| $F$ | F matrix |
| $F_T$ | thrust, $N$ |
| $f_{11}$, $f_{12}$, $f_{21}$, $f_{22}$ | F matrix entries |
| $g$ | gravity at major body, $m/s^2$ |
| $g_0$ | standard gravity, $m/s^2$  |
| $g_{eff}$ | effective gravity, $m/s^2$ |
| $I_{sp}$ | specific impulse, $s$ |
| $m$ | mass, $kg$ |
| $p_1$, $p_2$ | steering polynomials , <br> ($m/s^2$, $m/s$) |
| $\psi$ | heading, $rad.$ |
| $q$ | general distance coordinate, $m$ |
| $t$ | time, $s$ |
| $T$ | cutoff time, $s$ |
| $\tau$ | ($\equiv \frac{m_o}{\dot m}$), $s$ |
| $T_{go}$ | time-to-go, $s$ |
| $v$ | velocity, $m/s$ |
| $v_e$ | exhaust velocity, $m/s$ |

| Subscript | Description |
| ---   | ---   |
| $o$ | value at $t = t_o$ (the current time) |
| $D$ | desired value at $t = T$ (cut-off time) |
| $r$ | radial guidance constant |
| $y$ | plane control guidance constant |
| $peri$ | perifocal plane projection $\begin{bmatrix} \hat n & \hat e & 0 \end{bmatrix}$ |

## Appendix B: Reference Frames

### Global Frame

Axes are $\hat X$, $\hat Y$, $\hat Z$. Axes are inertial, origin is at center of the celestial body. In terms of (right ascension, declination), $\hat X$ is at $(0, 0)$, $\hat Y$ is at $(\frac{\pi}{2}, 0)$, and $\hat Z$ is at $(0, \frac{\pi}{2})$.

### Radial-Circumferential-Normal

$$\begin{align}
    \hat r = \frac{\vec r}{r} \tag{B.1} \\
    \hat h = \frac{\vec r \times \vec v}{|\vec r \times \vec v|} \tag{B.2} \\
    \hat \theta = \hat h \times \hat r \tag{B.3} 
\end{align}$$

### Perifocal

Origin is at center of celestial body. $\hat p$ points to orbit periapsis, $\hat q$ on orbital plane at $\nu = \frac{\pi}{2}$, and $\hat w = \hat p \times \hat q$.

The following defines the perifocal axes in terms of the global frame.

$$\begin{align}
    \begin{bmatrix}
        & & \\
        \hat p & \hat q & \hat w \\
        & &
    \end{bmatrix}
    = R_z(\Omega) R_x(i) R_z(\omega)
\end{align}$$

### Plane Control

$$\begin{align}
    \hat i = \frac{\vec r}{r} \tag{B.7} \\
    \hat j = \frac{\hat y \times \hat i}{|\hat y \times \hat i|} \tag{B.8} \\
    \hat k = \hat i \times \hat j \tag{B.9} 
\end{align}$$

Origin is at vehicle position $\vec r$. $\hat y$ is the normal vector of the target orbital plane, defined by $\Omega$ and $i$. Mainly used to simplify finding $v_\theta(T)$ and $\psi$.

### Topocentric

Origin is at vehicle position $\vec r$. $\hat n$ points North, $\hat e$ points East, $\hat d$ points downwards.

The following defines the topocentric axes in terms of the global frame.

$$\begin{align}
    \begin{bmatrix}
        & & \\
        \hat n & \hat e & \hat d \\
        & &
    \end{bmatrix}
    = R_z(RA) R_y(-DEC)
\end{align}$$

where $RA$ and $DEC$ are right ascension and declination, respectively.

## Appendix C: Abbreviated Derivation

The following is a derivation for the single-stage ascent guidance method implemented in this program. This is an abbreviated version of the derivation for fixed-thrust ascent guidance found in "A General, Explicit, Optimizing Guidance Law for Rocket-Propelled Spaceflight" by G. Cherry

In broad terms, the guidance method is derived by
- Applying the linear tangent law to the differential equation of radial motion, and to the differential equation of distance normal to the target orbital plane. This yields guidance laws for $\ddot r$ and $\ddot y$.
- Determining an expression for predicting $v_{\theta}(T)$ based on the guidance laws.
- Defining an iterative method for determining cut-off time $T$ based on the difference between the desired and predicted $v_\theta(T)$.
- Determining an iterative method for targeting a set of orbital elements.

This derivation is based completely on the paper by G.Cherry, with some modification:

1) Cherry derives the guidance law by defining the law to have the minimum number of terms necessary to uniquely satisfy the boundary conditions, and defers optimization of the guidance law to the paper's appendix. The derivation here instead starts from the content in Appendix A of Cherry, deriving the guidance laws based on an approximation of the linear tangent steering law and the differential equations of motion.
2) The method of predicting the final circumferential velocity derived in Cherry is a Taylor expansion, and it is designed exclusively for use with the radial guidance law. Here, the final circumferential velocity is predicted using a numerical integrator, and it incorporates both radial and plane control guidance.
3) The orbit targeting method in Cherry has an unconstrained argument of periapsis. Here, the argument of periapsis is given.

A numerical integrator is used here instead of a Taylor expansion mainly for convenience; the equation for $\dot v_{\theta}(t)$ derived here is large, and it was determined that a Taylor expansion would be much larger and more difficult to debug in the implemented program compared to using a numerical integrator.

### C.1. Fixed-Thrust Model

A constant-thrust model for a rocket, with constant mass flow and exhaust velocity, is defined as follows

$$\begin{align}
    v_e & = g_0I_{sp} = constant \tag{C.1.1} \\
    \dot{m} & = constant \tag{C.1.2} \\
    \dot{m} & > 0 \tag{C.1.3} \\
    F_T & = \dot{m} v_e \tag{C.1.4} 
\end{align}$$

The mass of the rocket vehicle is a linear function of time

$$\begin{align}
    m=m_o-\dot m t \tag{C.1.5}
\end{align}$$

where $m_o$ is the mass of the vehicle at $t=0$. Applying Newton's second law yields a formula for thrust acceleration

$$\begin{align}
    a_T=v_e/(\tau-t) \tag{C.1.6}
\end{align}$$

where

$$\begin{align}
    \tau \equiv m_o/ \dot m \tag{C.1.7}
\end{align}$$

$\tau$ can be interpreted as the time at which the rocket vehicle composed of only fuel and no structure will reach 0 mass. 

### C.2. Generalized Guidance Law

It will be helpful to have a general formula to solve the guidance laws that will be derived. A guidance law for generalized coordinate $q$ is defined:

$$\begin{align}
    \ddot q(t) & = c_1 p_1(t) + c_2 p_2(t) \tag{C.2.1} \\
\end{align}$$

where

$$\begin{align}
    p_1(t)&=a_T(t) \tag{C.2.2} \\
    p_2(t)&=(T-t)a_T(t) \tag{C.2.3} 
\end{align}$$

where $T$ is the time of guidance termination. The thrust acceleration $a_T$ is written in the following form to accomodate a Taylor expansion

$$\begin{align}
    a_T(t) = a_0 + a_1(T-t) + a_2(T-t)^2 + ... + a_n(T-t)^n \tag{C.2.4} \\
\end{align}$$

It is desirable to solve for $\ddot q(t)$. The constants $c_1$ and $c_2$ are unknown. $p_1(t)$ and $p_2(t)$ are given, and the following boundary conditions are provided

$$\begin{align}
    q_0 & = q(t_0) \tag{C.2.5} \\
    \dot q_0 & = \dot q(t_0) \tag{C.2.6} \\
    q_D & = q(T) \tag{C.2.7} \\
    \dot q_D & = \dot q(T) \tag{C.2.8} 
\end{align}$$

where $t_0$ is the current time. Integrating equation (C.2.1) yields the equations of constraint 

$$\begin{align}
    \dot q_D - \dot q_0 
        & = \int_{t_0}^T \ddot q(t) dt \tag{C.2.9}\\
        &= c_1 \int_{t_0}^T p_1(t) dt + c_2 \int_{t_0}^T p_2(t) dt \nonumber \\
    q_D - q_0 - \dot q(t_0)T_{go}
        & = \int_{t_0}^T \int_{t_0}^t \ddot q(s) ds \; dt \tag{C.2.10} \\
        &= c_1 \int_{t_0}^T \int_{t_0}^t p_1(s) ds \; dt
            + c_2 \int_{t_0}^T \int_{t_0}^t p_2(s) ds \; dt \nonumber
\end{align}$$

where

$$\begin{align}
    T_{go} = T - t_0 \tag{C.2.11} \\
\end{align}$$

The equations of constraint can be represented by the matrix equation

$$\begin{align}
    \begin{bmatrix}
    \dot q_D - \dot q_0 \\
    q_D - (q_o + \dot q_o T_{go})
    \end{bmatrix}
    = F 
    \begin{bmatrix}
    c_1 \\
    c_2
    \end{bmatrix} \tag{C.2.12}
\end{align}$$

where the $F$ matrix is a $2 \times 2$ matrix composed of the following entries

$$\begin{align}
    f_{11} & = a_0 T_{go} + a_1 T_{go}^2/2 + \dots + a_nT_{go}^{n+1}/(n+1) \tag{C.2.13} \\
    f_{12} & = a_0 T_{go}^2/2 + a_1 T_{go}^3/3 + \dots + a_nT_{go}^{n+2}/(n+2) \tag{C.2.14} \\
    f_{21} & = f_{12} \tag{C.2.15} \\
    f_{22} & = a_0 T_{go}^3/3 + a_1 T_{go}^4/4 + \dots + a_nT_{go}^{n+3}/(n+3) \tag{C.2.16}
\end{align}$$

$c_1$ and $c_2$ is solved from (C.2.12) by inverting the $F$ matrix. This solves the general guidance equation.

### C.3. Radial Guidance Law

The linear tangent steering law [2] will be used to derive the radial guidance law

$$\begin{align}
    \tan \alpha = A + Bt \tag{C.3.1} \\
\end{align}$$

The differential equation of radial motion is 

$$\begin{align}
    \ddot r = \vec a_T \cdot \hat r + g_{eff} = a_T \sin \alpha + g_{eff} \tag{C.3.2} \\
\end{align}$$

where

$$\begin{align}
    g_{eff} = -\mu/r^2 + v_{\theta}^2/r \tag{C.3.3} 
\end{align}$$

The tangent law can be approximated by 

$$\begin{align}
    \sin \alpha = A + Bt - g_{eff}/a_T \tag{C.3.4} \\
\end{align}$$

The approximation assumes that $\tan \alpha \approx \sin \alpha$, and that $g_{eff}/a_T \approx 0$. The latter approximation becomes more accurate near guidance termination: as the vehicle approaches the cut-off time $T$ when targeting a circular orbit, $g_{eff}$ approaches zero and $a_T$ continues increasing.

Substituting (C.3.4) into (C.3.2) yields

$$\begin{align}
    \ddot r = A a_T + B a_T t \tag{C.3.5} \\
\end{align}$$

If A and B are rewritten in terms of other constants as $A = c_{1,r} + c_{2,r} T$ and $B = -c_{2,r}$, then the equation can be written in the form of the generalized guidance law (C.2.1) as 

$$\begin{align}
    \ddot r = c_{1,r}p_1(t) + c_{2,r}p_2(t) \tag{C.3.6} \\
\end{align}$$

where

$$\begin{align}
    p_1(t) & = a_T \tag{C.3.7}  \\
    p_2(t) & = (T-t)a_T \tag{C.3.8} 
\end{align}$$

Given the following boundary conditions

$$\begin{align}
    r_0 & = r(t_0) \tag{C.3.9} \\
    \dot r_0 & = \dot r(t_0) \tag{C.3.10} \\
    r_D & = r(T) \tag{C.3.11} \\
    \dot r_D & = \dot r(T) \tag{C.3.12} 
\end{align}$$

$c_{1,r}$ and $c_{2,r}$ can be solved using the matrix form of the equations of constraint (C.2.12). This fully defines the guidance law for $\ddot r(t)$.

### C.4. Plane Control Guidance Law

The differential equation for $y$, the vehicle's distance from the target orbital plane along the plane's normal axis $\hat y$, is

$$\begin{align}
    \ddot y & = \vec a_T \cdot \hat y + \vec g \cdot \hat y \tag{C.4.1} \\
    & = a_T \sin \alpha_y + \vec g \cdot \hat y \notag\\
\end{align}$$

The linear tangent law can be approximated as

$$\begin{align}
    \sin \alpha_y(t) = A+Bt - \vec g \cdot \hat y/a_T \tag{C.4.2} \\
\end{align}$$

The approximation assumes that $\tan \alpha \approx \sin \alpha$, and that $(\vec g \cdot \vec y)/a_T \approx 0$. $\vec g \cdot \vec y$ is small in general, and $(\vec g \cdot \vec y)/a_T$ is even more so.

Substituting (C.4.2) in (C.4.1) yields 

$$\begin{align}
    \ddot y = A a_T + B a_T t \tag{C.4.3} \\
\end{align}$$

If the constants are written in terms of $c_{1,y}$ and $c_{2,y}$ as $A = c_{1,y} + c_{2,y} T$ and $B = -c_{2,y}$, then the equation can be written in the form of the generalized guidance law (C.2.1) as 

$$\begin{align}
    \ddot y = c_{1,y}p_1(t) + c_{2,y}p_2(t) \tag{C.4.4} \\
\end{align}$$

where

$$\begin{align}
    p_1(t) & = a_T \tag{C.4.5} \\
    p_2(t) & = (T-t)a_T \tag{C.4.6} 
\end{align}$$

Given the following boundary conditions

$$\begin{align}
    y_0 & = y(t_0) \tag{C.4.7} \\
    \dot y_0 & = \dot y(t_0) \tag{C.4.8} \\
    y_D & = y(T) \tag{C.4.9} \\
    \dot y_D & = \dot y(T) \tag{C.4.10} 
\end{align}$$

$c_{1,y}$ and $c_{2,y}$ can be solved using the matrix form of the equations of constraint (C.2.12). This fully defines the guidance law for $\ddot y(t)$.


### C.5. Time-to-Go

The radial guidance law and plane control guidance law require the cut-off time $T$ to be given in order to solve for the $c_1$ and $c_2$ constants. An iterative method of finding $T$ based on the target circumferential velocity $v_{\theta D}$ is derived.

The radial guidance law and plane control guidance law require the cut-off time $T$ to be given in order to solve for the $c_1$ and $c_2$ constants. An iterative method of finding $T$ based on the target circumferential velocity $v_{\theta D}$ is derived.

The differential equation for $\dot v_\theta$ is 

$$\begin{align}
    \dot v_\theta = a_T \cos \alpha - \frac{\dot r v_\theta}{r} \tag{C.5.1} 
\end{align}$$

$\dot v_\theta$ can be rewritten as 

$$\begin{align}
    \dot v_\theta = a_T - a_L \tag{C.5.2} 
\end{align}$$

where

$$\begin{align}
    a_L = (1 - \cos \alpha) a_T + \frac{\dot r v_\theta }{r} \tag{C.5.3} \\
\end{align}$$

Integrating (C.5.2) and solving for $T_{go}$ yields

$$\begin{align}
    T_{go} = \tau _o \{ 1 - \exp[-(v_{\theta D} - v_{\theta o} + \Delta v_{\theta L})/v_e] \} \tag{C.5.4} \\
\end{align}$$

where

$$\begin{align}
    \Delta v_{\theta L} = \int_{t_0}^T a_L(t) dt \tag{C.5.5} \\
\end{align}$$

The time-to-go $T_{go}$ is calculated using an iterative method based on varying guesses of $\Delta v_{\theta L}$. The formula for the next estimate of $\Delta v_{\theta L}$ based on the previous one is 

$$\begin{align}
    \Delta v_{\theta L, n+1} = v_{\theta D} - v_{\theta F, n} + \Delta v_{\theta L, n} \tag{C.5.6} 
\end{align}$$

where $v_{\theta D}$ is the target $v_\theta$ at time $T$, and $v_{\theta F, n}$ is the estimated $v_\theta(T)$ for thrust loss estimate $v_{\theta L, n}$. The following figure illustrates the procedure for calculating $\Delta v_{\theta L}$.
<p align="center">
    <img width="600px" src="iterative_T_go_algo.svg">
</p>


The estimates of $\Delta v_{\theta L}$ continue until the estimated final circumferential velocity is close enough to the desired final circumferential velocity

$$\begin{align}
    | v_{\theta D} - v_{\theta F, n} | < \epsilon \tag{C.5.7} 
\end{align}$$

where $\epsilon$ is the tolerable guidance scheme error.

The following rewrites the time-to-go equation (C.5.4) into the final form found in Cherry's [1] derivation, which is also the form currently used in the program
$$\begin{align}
    T_{go, n} = \tau_o \{1 - \exp [-(v_{\theta D} - v_{\theta o})/v_e]\, Q_n\} \tag{C.5.8} 
\end{align}$$

where
$$\begin{align}
    Q_{n} = \exp(-\Delta v_{\theta L, n}/v_e) \tag{C.5.9} \\
\end{align}$$

Equation (C.5.6) then becomes
$$\begin{align}
    Q_{n+1} = \exp \begin{bmatrix} \frac{-(v_{\theta D} - v_{\theta o})}{v_e} \end{bmatrix} \frac{Q_n}{H(T_n)} \tag{C.5.10} \\
\end{align}$$

where
$$\begin{align}
    H(T_n) = H_{F, n} = \exp[-(v_{\theta F, n} - v_{\theta o})/v_e] \tag{C.5.11} 
\end{align}$$

### C.6. Final Circumferential Velocity

A method for predicting the final circumferential velocity $v_{\theta}(T)$ is derived, to serve as a component of the iterative calculation for cut-off time $T$ in C.5.

The differential equation for circumferential velocity is
$$\begin{align}
    \dot v_{\theta}(t) = \vec a_T \cdot \hat \theta - \dot r v_\theta / r \tag{C.6.1}
\end{align}$$

$\dot v_{\theta}(t)$ will be numerically integrated from the initial condition $v_{\theta o}$ to find $v_{\theta}(T)$. Therefore, $v_\theta$ is assumed given. Expressions for $\vec a_T$, $\hat \theta$, $\dot r$, and $r$ must be found.

#### $\dot r(t)$ and $r(t)$

Both $\ddot r(t)$ and $\ddot y(t)$ are assumed given by the guidance laws.
$\dot r(t)$ and $r(t)$ are both found by integrating the radial guidance law (C.3.6) and using $t$ and $T$ as the boundary conditions
$$\begin{align}
    \begin{bmatrix}
        \dot r(t) \\
        r(t)
    \end{bmatrix} =
    \begin{bmatrix}
        \dot r_D \\
        r_D - \dot r(t) T_{go}
    \end{bmatrix}
    - F \begin{bmatrix}
        c_{1,r} \\
        c_{2, r}
    \end{bmatrix} \tag{C.6.2}
\end{align}$$

Similarly, a solution exists for $\dot y(t)$ and $y(t)$ when integrating (C.4.4)
$$\begin{align}
    \begin{bmatrix}
        \dot y(t) \\
        y(t)
    \end{bmatrix} =
    \begin{bmatrix}
        0 \\
        - \dot y(t) T_{go}
    \end{bmatrix}
    - F \begin{bmatrix}
        c_{1,y} \\
        c_{2, y}
    \end{bmatrix} \tag{C.6.3}
\end{align}$$

#### $\hat \theta(t)$

The circumferential unit vector $\hat \theta$ is given by 

$$\begin{align}
    \hat \theta(t) = \frac{\vec v_\theta(t)}{v_\theta(t)} \tag{C.6.7}
\end{align}$$

By definition, circumferential velocity $v_\theta$ in PCF axes has no $\hat i$ component ($\hat \theta$ is orthogonal to $\hat r$)

$$\begin{align}
    \vec v_\theta = \begin{bmatrix} 0 & v_{j} & v_{ k} \end{bmatrix} \tag{C.6.8}
\end{align}$$

The $\hat k$ component of velocity will be calculated from the commanded velocity projected along the $\hat y$ axis
$$\begin{align}
    \vec v \cdot \hat y = \dot y \tag{C.6.9}
\end{align}$$

where $\dot y$ is given by (C.6.3). Define $\beta (t)$ as the angle of the vehicle's position with respect to the target orbital plane.
<p align="center">
    <img src="B6_beta_angle.svg">
</p>

Based on the figure, $\hat y$ can be written in PCF axes as
$$\begin{align}
    \hat y = \begin{bmatrix}
        y/r \\
        0 \\
        \sqrt{r^2 + y^2}/r
    \end{bmatrix} \tag{C.6.10}
\end{align}$$

Substituting (C.6.10) into (C.6.9) and solving for $v_{k}$ yields

$$\begin{align}
    v_{ k} & = \frac{\dot y - v_{i} \hat y_{i}}{\hat y_{k}} \tag{C.6.11} \\
    & = \frac{\dot y - \dot r \hat y_{i}}{\hat y_{k}} \notag
\end{align}$$

$v_{\hat j}$ is given by finding the portion of velocity "unused" by the $\hat i$ and $\hat k$ components of velocity
$$\begin{align}
    v_{\hat j} = \sqrt{v^2 - v_{\hat i}^2 - v_{\hat k}^2} \tag{C.6.12}
\end{align}$$

Equation (C.6.7) can now be solved to yield $\hat \theta(t)$.

#### $\vec a_T(t)$

$a_T$ is given by 
$$\begin{align}
    \vec a_T = \begin{bmatrix} a_{T \hat i} & a_{T \hat j} & a_{T \hat k} \end{bmatrix} \tag{C.6.13}
\end{align}$$

$\vec a_T \cdot \hat i$ is given by rearranging the differential equation for radial motion (C.3.2)
$$\begin{align}
    \vec a_T \cdot \hat i = \vec a_T \cdot \hat r =  \ddot r - g_{eff}  \tag{C.6.14}
\end{align}$$

$\vec a_T \cdot \hat k$ is found by substituting the expression for $\hat y$ (C.6.10) into the differential equation for $\ddot y$ (C.4.1)
$$\begin{align}
    \vec a_T \cdot \hat k = \frac{\vec a_T \cdot \hat y - (\vec a_T \cdot \hat r)(\hat y_{\hat i})}{\hat y_{\hat k}} \tag{C.6.15}
\end{align}$$

where
$$\begin{align}
    \vec a_T \cdot \hat y & = \ddot y - \vec g \cdot \hat y \tag{C.6.16} \\
    & = \ddot y + \frac{\mu y}{r^3} \notag
\end{align}$$

$\vec a_T \cdot \hat j$ is given by
$$\begin{align}
    \vec a_T \cdot \hat j = \sqrt{a_T^2 - (\vec a_T \cdot \hat i)^2 - (\vec a_T \cdot \hat k)} \tag{C.6.17}
\end{align}$$

All the components of $\vec a_T(t)$ have been found.

### C.7. Pitch and Heading

The pitch and heading commands are found using the commanded thrust acceleration $a_T(t)$ (C.6.13). 

$$\begin{align}
    \alpha & = \sin^{-1}(\frac{\vec a_T \cdot \hat r}{a_T}) \tag{C.7.1} \\
    \psi & = \arctan2(a_{T_e},\, a_{T_n}) \tag{C.7.2}
\end{align}$$

(C.7.2) uses the topocentric coordinates of $a_T$.

### C.8. Final True Anomaly

Finding the true anomaly $\nu(T)$ at cutoff time is necessary for targeting a series of orbital elements. The true anomaly at cutoff is with respect to the target orbit, but the true anomaly of the vehicle during ascent is with respect to an orbit that is constantly changing. Therefore, for the purpose of this calculation, it is assumed that the true anomaly of the launch vehicle at any point is given by its projection onto the perifocal plane of its target orbit.

$$\begin{align}
    \vec r_{peri} = r_q \hat q + r_p \hat p \tag{C.8.1} \\
    \nu_{peri} = atan2(r_{q}, r_{p}) \tag{C.8.2} 
\end{align}$$

where $\hat p$, $\hat q$, $\hat w$ are the perifocal axes.

The differential equation for $\dot \nu_{peri}$ is
$$\begin{align}
    \dot \nu_{peri} = \frac{\vec v \cdot \hat \theta_{peri}}{r_{peri}} \tag{C.8.3} 
\end{align}$$

Where $\hat \theta_{peri}$ is the circumferential unit vector at $\vec r_{peri}$. $\hat \theta_{peri} = \hat j$, so $\dot \nu_{peri}$ is rewritten as

$$\begin{align}
    \dot \nu_{peri} = \frac{\vec v \cdot \hat j}{r_{peri}} \tag{C.8.4} 
\end{align}$$

Numerically integrating this from $t_o$ to $T$ yields

$$\begin{align}
    \nu_{peri}(T) = \Delta \nu_{peri}(T) + \nu_{peri}(t_o) \tag{C.8.5} 
\end{align}$$

where 

$$\begin{align}
    \Delta \nu_{peri}(T) = \int_{t_o}^T \dot \nu_{peri}(t)\; dt \tag{C.8.6}
\end{align}$$

(C.8.5) is used to predict the true anomaly at burnout.

### C.9. Orbit Targeting

The target orbit is described by 
$$
\begin{matrix} 
r_p& r_a & i & \Omega & \omega \tag{C.9.1}
\end{matrix}
$$

The variables required by the algorithm are
$$\begin{matrix}
    r_D & \dot r_D & v_\theta & i & \Omega \tag{C.9.2}
\end{matrix}$$

The variables $r_p$, $r_a$, and $\omega$ must be converted into $r_D$, $\dot r_D$, and $v_{\theta D}$ to be used in the radial guidance law, and in the calculation of cut-off time.

True anomaly at cut-off $\nu(T) = \nu_{proj}(T)$ is given by (C.8.5).

Intermediate orbit values are calculated
$$\begin{align}
    a = \frac{r_p + r_a}{2} \tag{C.9.3} \\
    e = 1 - \frac{r_p}{a} \tag{C.9.4} \\
    h = \sqrt{r_p \mu (1+e)} \tag{C.9.5} \\
\end{align}$$

The desired values are found
$$\begin{align}
    r_D = a \frac{(1-e^2)}{1 + e\cos(\nu)} \tag{C.9.6} \\
    v_{r D} = \mu/h e \sin(\nu) \tag{C.9.7} \\
    v_{\theta D} = \frac{h}{r} \tag{C.9.8}
\end{align}$$

The orbit targeting is placed outside the time-to-go calculation loop and solved iteratively until the change of $\nu(T)$ between iterations becomes smaller than a certain error value.


## References
[1] G. W. Cherry, "A General, Explicit, Optimizing Guidance Law for Rocket-Propelled Spaceflight," in *Astrodynamics Guidance and Control Conference, August 24-26, 1964, Los Angeles, CA, USA* [Online]. Available: ARC, https://arc.aiaa.org/doi/10.2514/6.1964-638

[2] A.E. Bryson, Jr. and Y. Ho, "Optimization Problems for Dynamic Systems," in *Applied Optimal Control,* Waltham, MA, USA: Ginn, 1969, pp. 61.