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
| $a_0$, $a_1$, ... $a_n$ |  |
| $a_T$ | thrust acceleration, $m/s^2$ |
| $c_1$, $c_2$ | steering constants (eq. (B.2.1)), <br> (nondimensional, $s^{-1}$) |
| $F$ | ??? |
| $F_T$ | thrust, $N$ |
| $g_0$ | standard gravity, $m/s^2$  |
| $I_{sp}$ | specific impulse, $s$ |
| $m$ | mass, $kg$ |
| $p_1$, $p_2$ | steering polynomials (eq. (B.2.2-3)), <br> ($m/s^2$, $m/s$) |
| $q$ | general distance coordinate, $m$ |
| $T$ | cutoff time, $s$ |
| $\tau$ | (eq. (B.1.7)), $s$ |
| $T_{go}$ | time-to-go, $s$ |
| $v_e$ | exhaust velocity, $m/s$ |

subscript $o$ indicates current time, except for $m_o$? Which indicates mass at time $t=0$.


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

where $t_0$ is the current time. Integrating equation (B.2.1) yields the equations of constraint 
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

The LINEAR TANGENT LAW ...

## References
[1] G. W. Cherry, "A General, Explicit, Optimizing Guidance Law for Rocket-Propelled Spaceflight," in *Astrodynamics Guidance and Control Conference, August 24-26, 1964, Los Angeles, CA, USA* [Online]. Available: ARC, https://arc.aiaa.org/doi/10.2514/6.1964-638

TODO: Perhaps tone down the formal reference.

## Notes
From Cherry[1], page 4: "Explicit guidance laws are laws which express the formulas for the steering commands directly in terms of the current and desired boundary values of the components of the position and velocity vectors. For the guidance laws to be truly explicit, that is valid for any values of the current and desired boundary conditions, the laws must be derived as direct solutions to the equations of motion."

Why did I implement VThetaSolver instead of using the integrated pitch heading query directly, since I was going to integrate anyways? The equation looks more complicated that the pitch heading query one.