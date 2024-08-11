## TODOs
- How to differentiate between referring to gcherry program vs referring to the paper.
    - maybe just refer to it as the 'program' or the 'implementation'
- Define symbols for orbital elements
- Review assertion made at beginning of abbreviated derivation
- Finish VTheta and time to go components
- Figure out values for beginning/final coordinates, target v_{\theta}.
- Add figure 9?
- Choose where to add fractions vs inline division
    - just do it willy nilly, worry about it next revision.
- Choose when to add time vs not.
    - just do it willy nilly, worry about it next revision.
- Add figure for plane control frame.

# Table of Contents
1. [test1](#appendix-a-symbols)


## Appendix A: Symbols

$a_T$
$a_0$, $a_1$, $a_2$
$y$
$v_e$
F_T
m
m_o
t
tau
q
c_1, c_2
c_1,radial, c_2,radial
c_1,yaw, c_2,yaw
p_1, p_2
T
T_{go}
\alpha
g_{eff}
\mu
v_{theta}
r
A
B
C
D
\vec g
\vec y
a_L
\Delta v_{\theta L}
\epsilon
\tau_o
Q
H
\beta
\hat y

superscripts

orbital elements

???
\hat i, \hat j, \hat k
??? Do we want to keep these symbols for PCF, or use something else?
| Symbol | Description |
| ---   | ---   |
| $A$, $B$ | |
| $a_0$, $a_1$, ... $a_n$ |  |
| $a_T$ | thrust acceleration, $m/s^2$ |
| $\alpha$ |  |
| $\alpha_y$ |  |
| $c_1$, $c_2$ | steering constants (eq. (B.2.1)), <br> (nondimensional, $s^{-1}$) || $F$ | ??? |
| $F$ | F matrix (eq. (B.2.12)) |
| $F_T$ | thrust, $N$ |
| $f_{11}$, $f_{12}$, $f_{21}$, $f_{22}$ | F matrix entries (eq. (B.2.13-16)) |
| $g$ | gravity at major body, $m/s^2$ |
| $g_0$ | standard gravity, $m/s^2$  |
| $g_{eff}$ | effective gravity (eq. (B.3.3)), $m/s^2$ |
| $I_{sp}$ | specific impulse, $s$ |
| $m$ | mass, $kg$ |
| $p_1$, $p_2$ | steering polynomials (eq. (B.2.2-3)), <br> ($m/s^2$, $m/s$) |
| $q$ | general distance coordinate, $m$ |
| $t$ |  |
| $T$ | cutoff time, $s$ |
| $\tau$ | (eq. (B.1.7)), $s$ |
| $T_{go}$ | time-to-go (eq. (B.2.11)), $s$ |
| $\theta_{pitch}$ |  |
| $v
| $v_e$ | exhaust velocity, $m/s$ |

subscript $o$ indicates current time, except for $m_o$? Which indicates mass at time $t=0$.
or $0$ indicates current time.
$D$ indicates value at $t=T$
subscript $r$ for radial steering constants
subscript $y$ for plane control steering constants

B.1
B.2
B.3
B.4


## Appendix B: Abbreviated Derivation

The following is a derivation of an iterative method for guiding a single-stage rocket ascent vehicle. This derivation is based completely on that of Cherry [1], with some modification:

1) Cherry derives the guidance law by defining the law to have the minimum number of terms necessary to uniquely satisfy the boundary conditions, and defers optimization of the guidance law to the appendix. The derivation here instead starts from the content in Appendix A, and derives the guidance laws based on an approximation of the linear tangent steering law and the differential equations of motion.
2) The method of predicting the final circumferential velocity derived in Cherry is a Taylor expansion and it uses the radial guidance law. Here, the final circumferential velocity is predicted using a numerical integrator, and incorporates both radial and plane control guidance. A numerical integrator is used here instead of a Taylor expansion mainly for convenience; the equation for $\dot v_{\theta}(t)$ derived here is large, and it was determined that a Taylor expansion would be much larger and more difficult to debug in the implemented program compared to using a numerical integrator.
3) The orbit targeting method in Cherry has an arbitary argument of periapsis, while the orbit targeting method here specifies the argument of periapsis.

In broad terms, the guidance method is derived by
1) Applying the linear tangent law [2] to the differential equation of radial motion, and to the differential equation of distance normal to the target orbital plane. This yields guidance laws for $\ddot r$ and $\ddot y$. 
2) Finding a method for estimating the final value for circumferential velocity.
3) Solving for $T_{go}$ using the differential equation for circumferential velocity, and defining an iterative method of solving $T_{go}$ based on estimated values of the final circumferential velocity $v_{\theta}(T)$.

### B.1. Fixed-Thrust Model

A constant-thrust model for a rocket, with constant mass flow and exhaust velocity, is defined as follows

$$\begin{align}
    v_e & = g_0I_{sp} = constant \tag{B.1.1} \\
    \dot{m} & = constant \tag{B.1.2} \\
    \dot{m} & > 0 \tag{B.1.3} \\
    F_T & = \dot{m} v_e \tag{B.1.4} 
\end{align}$$

The mass of the rocket vehicle is a linear function of time

$$\begin{align}
    m=m_o-\dot m t \tag{B.1.5}
\end{align}$$

where $m_o$ is the mass of the vehicle at $t=0$. Applying Newton's second law yields a formula for thrust acceleration

$$\begin{align}
    a_T=v_e/(\tau-t) \tag{B.1.6}
\end{align}$$

where

$$\begin{align}
    \tau \equiv m_o/ \dot m \tag{B.1.7}
\end{align}$$

$\tau$ can be interpreted as the time at which the rocket vehicle composed of only fuel (no structure) will reach 0 mass. 


### B.2. Generalized Guidance Law

It will be helpful to have a general formula to solve for the guidance laws that will be derived. A guidance law for generalized coordinate $q$ is defined:

$$\begin{align}
    \ddot q(t) & = c_1 p_1(t) + c_2 p_2(t) \tag{B.2.1} \\
\end{align}$$

where

$$\begin{align}
    p_1(t)&=a_T(t) \tag{B.2.2} \\
    p_2(t)&=(T-t)a_T(t) \tag{B.2.3} 
\end{align}$$

where $T$ is the time of guidance termination and the thrust acceleration $a_T$ is written in the form

$$\begin{align}
    a_T(t) = a_0 + a_1(T-t) + a_2(T-t)^2 + ... + a_n(T-t)^n \tag{B.2.4} \\
\end{align}$$

It is desirable to solve for $\ddot q(t)$. The constants $c_1$ and $c_2$ are unknown. $p_1(t)$ and $p_2(t)$ are given, and the following boundary conditions are provided

$$\begin{align}
    q_0 & = q(t_0) \tag{B.2.5} \\
    \dot q_0 & = \dot q(t_0) \tag{B.2.6} \\
    q_D & = q(T) \tag{B.2.7} \\
    \dot q_D & = \dot q(T) \tag{B.2.8} 
\end{align}$$

where $t_0$ is the current time. Integrating the equation (B.2.1) yields the equations of constraint 

$$\begin{align}
    \dot q_D - \dot q_0 
        & = \int_{t_0}^T \ddot q(t) dt \tag{B.2.9}\\
        &= c_1 \int_{t_0}^T p_1(t) dt + c_2 \int_{t_0}^T p_2(t) dt \nonumber \\
    q_D - q_0 - \dot q(t_0)T_{go}
        & = \int_{t_0}^T \int_{t_0}^t \ddot q(s) ds \; dt \tag{B.2.10} \\
        &= c_1 \int_{t_0}^T \int_{t_0}^t p_1(s) ds \; dt
            + c_2 \int_{t_0}^T \int_{t_0}^t p_2(s) ds \; dt \nonumber
\end{align}$$

where

$$\begin{align}
    T_{go} = T - t_0 \tag{B.2.11} \\
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
    \end{bmatrix} \tag{B.2.12}
\end{align}$$

where the $F$ matrix is a $2 \times 2$ matrix composed of the following entries

$$\begin{align}
    f_{11} & = a_0 T_{go} + a_1 T_{go}^2/2 + \dots + a_nT_{go}^{n+1}/(n+1) \tag{B.2.13} \\
    f_{12} & = a_0 T_{go}^2/2 + a_1 T_{go}^3/3 + \dots + a_nT_{go}^{n+2}/(n+2) \tag{B.2.14} \\
    f_{21} & = f_{12} \tag{B.2.15} \\
    f_{22} & = a_0 T_{go}^3/3 + a_1 T_{go}^4/4 + \dots + a_nT_{go}^{n+3}/(n+3) \tag{B.2.16}
\end{align}$$

$c_1$ and $c_2$ can be solved from the matrix equation by inverting the $F$ matrix. This solves the general guidance equation.


### B.3. Radial Guidance Law

The linear tangent steering law [2] will be used to derive the radial guidance law

$$\begin{align}
    \tan \alpha = A + Bt \tag{B.3.1} \\
\end{align}$$

The differential equation of radial motion is 

$$\begin{align}
    \ddot r = \vec a_T \cdot \hat r + g_{eff} = a_T \sin \alpha + g_{eff} \tag{B.3.2} \\
\end{align}$$

where

$$\begin{align}
    g_{eff} = -\mu/r^2 + v_{\alpha}^2/r \tag{B.3.3} 
\end{align}$$

The tangent law can be approximated by 

$$\begin{align}
    \sin \alpha = A + Bt - g_{eff}/a_T \tag{B.3.4} \\
\end{align}$$

The approximation assumes that $\tan \alpha \approx \sin \alpha$, and that $g_{eff}/a_T \approx 0$. The latter approximation becomes more accurate near guidance termination: as the vehicle approaches the cut-off time $T$ when targeting a circular orbit, $g_{eff}$ approaches zero and $a_T$ continues increasing.

Substituting (B.3.4) into (B.3.2) yields

$$\begin{align}
    \ddot r = A a_T + B a_T t \tag{B.3.5} \\
\end{align}$$

If it is assumed $A = c_{1,r} + c_{2,r} T$ and $B = -c_{2,r}$ (rewriting constants in terms of other constants), then the equation can be written in the form of the generalized guidance law (B.2.1) as 

$$\begin{align}
    \ddot r = c_{1,r}p_1(t) + c_{2,r}p_2(t) \tag{B.3.6} \\
\end{align}$$

where

$$\begin{align}
    p_1(t) & = a_T \tag{B.3.7}  \\
    p_2(t) & = (T-t)a_T \tag{B.3.8} 
\end{align}$$

Given the following boundary conditions

$$\begin{align}
    r_0 & = r(t_0) \tag{B.3.9} \\
    \dot r_0 & = \dot r(t_0) \tag{B.3.10} \\
    r_D & = r(T) \tag{B.3.11} \\
    \dot r_D & = \dot r(T) \tag{B.3.12} 
\end{align}$$

$c_{1,r}$ and $c_{2,r}$ can be solved using the matrix form of the equations of constraint (B.2.12). This fully defines the guidance law for $\ddot r(t)$.

### B.4. Plane Control Guidance Law

The differential equation for $y$, the vehicle's distance from the target orbital plane along the plane's normal axis $\hat y$, is

$$\begin{align}
    \ddot y & = \vec a_T \cdot \hat y + \vec g \cdot \hat y \tag{B.4.1} \\
    & = a_T \sin \alpha_y + \vec g \cdot \hat y \notag\\
\end{align}$$

The linear tangent law can be approximated as

$$\begin{align}
    \sin \alpha_y(t) = A+Bt - \vec g \cdot \hat y/a_T \tag{B.4.2} \\
\end{align}$$

The approximation assumes that $\tan \alpha \approx \sin \alpha$, and that $(\vec g \cdot \vec y)/a_T \approx 0$. $\vec g \cdot \vec y$ is small in general, and $(\vec g \cdot \vec y)/a_T$ even more so.

Substituting (B.4.2) in (B.4.1) yields 

$$\begin{align}
    \ddot y = A a_T + B a_T t \tag{B.4.3} \\
\end{align}$$

If the constants are written in terms of $c_{1,y}$ and $c_{2,y}$ as $A = c_{1,y} + c_{2,y} T$ and $B = -c_{2,y}$, then the equation can be written in the form of the generalized guidance law (B.2.1) as 

$$\begin{align}
    \ddot y = c_{1,y}p_1(t) + c_{2,y}p_2(t) \tag{B.4.4} \\
\end{align}$$

where

$$\begin{align}
    p_1(t) & = a_T \tag{B.4.5} \\
    p_2(t) & = (T-t)a_T \tag{B.4.6} 
\end{align}$$

Given the following boundary conditions

$$\begin{align}
    y_0 & = y(t_0) \tag{B.4.7} \\
    \dot y_0 & = \dot y(t_0) \tag{B.4.8} \\
    y_D & = y(T) \tag{B.4.9} \\
    \dot y_D & = \dot y(T) \tag{B.4.10} 
\end{align}$$

$c_{1,y}$ and $c_{2,y}$ can be solved using the matrix form of the equations of constraint (B.2.12). This fully defines the guidance law for $\ddot y(t)$.


### B.5. Time-To-Go

The radial guidance law and plane control guidance law require the cut-off time $T$ to be given in order to solve for the constants. An iterative method of finding $T$ based on the target circumferential velocity $v_{\theta D}$ is derived.

The differential equation for $\dot v_\theta$ is 

$$\begin{align}
    \dot v_\theta = a_T \cos \alpha - \frac{\dot r v_\theta}{r} \tag{B.5.1} 
\end{align}$$

$\dot v_\theta$ can be rewritten as 

$$\begin{align}
    \dot v_\theta = a_T - a_L \tag{B.5.2} 
\end{align}$$

where

$$\begin{align}
    a_L = (1 - \cos \alpha) a_T + \frac{\dot r v_\theta }{r} \tag{B.5.3} \\
\end{align}$$

Integrating (B.5.2) and solving for $T_{go}$ yields

$$\begin{align}
    T_{go} = \tau _o \{ 1 - \exp[-(v_{\theta D} - v_{\theta o} + \Delta v_{\theta L})/v_e] \} \tag{B.5.4} \\
\end{align}$$

where

$$\begin{align}
    \Delta v_{\theta L} = \int_{t_0}^T a_L(t) dt \tag{B.5.5} \\
\end{align}$$

The time-to-go $T_{go}$ is calculated using an iterative method based on varying guesses of $\Delta v_{\theta L}$. The formula for the next estimate of $\Delta v_{\theta L}$ based on the previous one is 

$$\begin{align}
    \Delta v_{\theta L, n+1} = v_{\theta D} - v_{\theta F, n} + \Delta v_{\theta L, n} \tag{B.5.6} 
\end{align}$$

where $v_{\theta D}$ is the target $v_\theta$ at time $T$, and $v_{\theta F, n}$ is the estimated $v_\theta(T)$ for thrust loss estimate $v_{\theta L, n}$. The following figure illustrates the procedure for calculating $\Delta v_{\theta L}$.
<p align="center">
    <img width="600px" src="iterative_T_go_algo.svg">
</p>


The estimates of $\Delta v_{\theta L}$ continue until the estimated final circumferential velocity is close enough to the desired final circumferential velocity

$$\begin{align}
    | v_{\theta D} - v_{\theta F, n} | < \epsilon \tag{B.5.7} 
\end{align}$$

where $\epsilon$ is the tolerable guidance scheme error.

The following rewrites the time-to-go equation into the final form found in Cherry's [1] derivation, which is also the form in the IMPLEMENTATION.
$$\begin{align}
    T_{go, n} = \tau_o \{1 - \exp [-(v_{\theta D} - v_{\theta o})/v_e]\, Q_n\} \tag{B.5.8} 
\end{align}$$

where
$$\begin{align}
    Q_{n} = \exp(-\Delta v_{\theta L, n}/v_e) \tag{B.5.9} \\
\end{align}$$

Equation (B.5.6) then becomes
$$\begin{align}
    Q_{n+1} = \exp \begin{bmatrix} \frac{-(v_{\theta D} - v_{\theta o})}{v_e} \end{bmatrix} \frac{Q_n}{H(T_n)} \tag{B.5.10} \\
\end{align}$$

where
$$\begin{align}
    H(T_n) = H_{F, n} = \exp[-(v_{\theta F, n} - v_{\theta o})/v_e] \tag{B.5.11} 
\end{align}$$

### B.6. Final Circumferential Velocity

A method for predicting the final circumferential velocity $v_{\theta}(T)$ is derived, to serve as a component of the iterative calculation for cut-off time $T$ in B.5.

(This also calculated $\Delta \theta(T)$)

The differential equation for circumferential velocity is
$$\begin{align}
    \dot v_{\theta}(t) = \vec a_T \cdot \hat \theta - \dot r v_\theta / r \tag{B.6.1}
\end{align}$$

$\dot v_{\theta}(t)$ will be numerically integrated to find $v_{\theta}(T)$. In this scheme, $v_\theta$ is given. Expressions for $\vec a_T$, $\hat \theta$, $\dot r$, and $r$ must be found.

#### B.6.1. $\dot r(t)$ and $r(t)$

Both $\ddot r(t)$ and $\ddot y(t)$ are assumed given by the guidance laws.
$\dot r(t)$ and $r(t)$ are both found by integrating the radial guidance law (B.3.6) and using T as one of the boundary conditions
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
    \end{bmatrix} \tag{B.6.2}
\end{align}$$

Similarly, a solution exists for $\dot y(t)$ and $y(t)$ when integrating (B.4.4)
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
    \end{bmatrix} \tag{B.6.3}
\end{align}$$

#### B.6.2. PCF

The components of $\hat \theta(t)$ and $a_T(t)$ in the following sub-sections will be defined in terms of the Plane Control Frame (PCF). The axes of PCF are defined as
$$\begin{align}
    \hat i & = \hat r  \tag{B.6.4} \\
    \hat j & = \frac{\hat y \times \hat i}{|\hat y \times \hat i |} \tag{B.6.5} \\
    \hat k & = \hat i \times \hat j \tag{B.6.6}
\end{align}$$
and $\hat y$ is the normal vector of the target orbital plane. The frame has origin at vehicle position $\vec r$.


#### B.6.3. $\hat \theta(t)$

- Find $\hat \theta(t) = \vec v_\theta(t)/v_\theta(t)$
    - $v_\theta(t)$ given
    - Find $\vec v_\theta(t)$
        - $\vec v_\theta = \begin{bmatrix} 0 & v_{\hat j} & v_{\hat k} \end{bmatrix}$
        - Find $v_{\theta \hat k}$
            - $\hat y(t) = \begin{bmatrix} \sin \beta & 0 & \cos \beta \end{bmatrix} $
        - Find $v_{\theta \hat j}$
            - Dependent on $v_{\theta \hat i}$, which is given by guidance
            - Depends on $v_{\theta \hat j}$ and $v_{\theta \hat k}$        

The circumferential unit vector $\hat \theta$ is given by 
$$\begin{align}
    \hat \theta(t) = \frac{\vec v_\theta(t)}{v_\theta(t)} \tag{B.6.7}
\end{align}$$

Circumferential velocity $v_\theta$ in PCF axes is by definition
$$\begin{align}
    \vec v_\theta = \begin{bmatrix} 0 & v_{\hat j} & v_{\hat k} \end{bmatrix} \tag{B.6.8}
\end{align}$$

The $\hat k$ component of velocity is calculated from the commanded velocity along the $\hat y$ axes
$$\begin{align}
    \vec v \cdot \hat y = \dot y \tag{B.6.9}
\end{align}$$

where $\dot y$ is calculated from the yaw guidance law. Define $\beta (t)$ as the angle of the vehicle's position with respect to the target orbital plane.
<p align="center">
    <img src="B6_beta_angle.svg">
</p>

Based on the figure, $\hat y$ can be written in PCF axes as
$$\begin{align}
    \hat y = \begin{bmatrix}
        y/r \\
        0 \\
        \sqrt{r^2 + y^2}/r
    \end{bmatrix} \tag{B.6.10}
\end{align}$$

Substituting (B.6.10) into (B.6.9) and solving for $v_{\hat k}$ yields

$$\begin{align}
    v_{\hat k} & = \frac{\dot y - v_{\hat i} \hat y_{\hat i}}{\hat y_{\hat k}} \tag{B.6.11} \\
    & = \frac{\dot y - \dot r \hat y_{\hat i}}{\hat y_{\hat k}} \notag
\end{align}$$

$v_{\hat j}$ is given by finding the portion of velocity "unused" by the $\hat i$ and $\hat k$ components of velocity
$$\begin{align}
    v_{\hat j} = \sqrt{v^2 - v_{\hat i}^2 - v_{\hat k}^2} \tag{B.6.12}
\end{align}$$

Equation (B.6.7) can now be solved to yield $\hat \theta(t)$.


#### B.6.4. $\vec a_T(t)$

- Find $\vec a_T = \begin{bmatrix} a_{T \hat i} & a_{T \hat j} & a_{T \hat k} \end{bmatrix}$
    - Find $a_{T \hat i}$
        - $a_{T \hat i} = \ddot r - g_{eff}$
            - $g_{eff} = -\mu/r^2 + v_{\theta}^2/r$
    - Find $a_{T \hat k}$
        - $a_{T} \cdot \hat y = \ddot y - \vec g \cdot \hat y$
        - based on $a_{T \hat i} as well
    - Find $a_{T \hat j}$

$a_T$ is given by 
$$\begin{align}
    \vec a_T = \begin{bmatrix} a_{T \hat i} & a_{T \hat j} & a_{T \hat k} \end{bmatrix} \tag{B.6.13}
\end{align}$$

$\vec a_T \cdot \hat i$ is given by rearranging the differential equation for radial motion (B.3.2)
$$\begin{align}
    \vec a_T \cdot \hat i = \vec a_T \cdot \hat r =  \ddot r - g_{eff}  \tag{B.6.14}
\end{align}$$

$\vec a_T \cdot \hat k$ is found by substituting the expression for $\hat y$ (B.6.10) into the differential equation for $\ddot y$ (B.4.1)
$$\begin{align}
    \vec a_T \cdot \hat k = \frac{\vec a_T \cdot \hat y - (\vec a_T \cdot \hat r)(\hat y_{\hat i})}{\hat y_{\hat k}} \tag{B.6.15}
\end{align}$$

where
$$\begin{align}
    \vec a_T \cdot \hat y & = \ddot y - \vec g \cdot \hat y \tag{B.6.16} \\
    & = \ddot y + \frac{\mu y}{r^3} \notag
\end{align}$$

$\vec a_T \cdot \hat j$ is given by
$$\begin{align}
    \vec a_T \cdot \hat j = \sqrt{a_T^2 - (\vec a_T \cdot \hat i)^2 - (\vec a_T \cdot \hat k)} \tag{B.6.17}
\end{align}$$

Equation (B.6.13) can now be solved to yield $\vec a_T(t)$

## References
[1] G. W. Cherry, "A General, Explicit, Optimizing Guidance Law for Rocket-Propelled Spaceflight," in *Astrodynamics Guidance and Control Conference, August 24-26, 1964, Los Angeles, CA, USA* [Online]. Available: ARC, https://arc.aiaa.org/doi/10.2514/6.1964-638

[2] A.E. Bryson, Jr. and Y. Ho, "Optimization Problems for Dynamic Systems," in *Applied Optimal Control,* Waltham, MA, USA: Ginn, 1969, pp. 61.

TODO: Perhaps tone down the formal reference.

## Notes
From Cherry[1], page 4: "Explicit guidance laws are laws which express the formulas for the steering commands directly in terms of the current and desired boundary values of the components of the position and velocity vectors. For the guidance laws to be truly explicit, that is valid for any values of the current and desired boundary conditions, the laws must be derived as direct solutions to the equations of motion."

Why did I implement VThetaSolver instead of using the integrated pitch heading query directly, since I was going to integrate anyways? The equation looks more complicated that the pitch heading query one.

I think $a_T$ is defined using a Taylor expansion since the integral for $a_T$ yields a logarithm, which may take 30 times the amount of time to multiply. I generally do not care about the performance here.

Orbital elements: $a$, $e$, $i$, $\Omega$, $\omega$, $\nu$

