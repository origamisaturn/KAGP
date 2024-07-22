# GCherry Implementation
## Guidance Objects
## Simulation Objects
### IntegratorSim
### KRPCClient
## Guidance Components
### OrbitTargetingAscent
### DebugAscent1
## Appendix A: Symbols

## Appendix B: Abbreviated Derivation
### Fixed Thrust
A constant thrust model for a rocket, with constant mass flow and constant exhaust velocity, is defined as follows
$$\begin{align}
    v_e & = gI_{sp} = constant \\
    \dot{m} & = constant \\
    \dot{m} & > 0 \\
    F_T & = \dot{m} v_e
\end{align}$$
The mass of the rocket vehicle is a linear function of time
$$\begin{align}
    m=m_o-\dot m t
\end{align}$$
This assumes that $m_o$ is the mass of the vehicle at $t=0$. Applying Newton's second law yields a formula for thrust acceleration
$$\begin{align}
    a_T=v_e/(\tau-t)
\end{align}$$
where 
$$\begin{align}
    \tau \equiv m_o/ \dot m
\end{align}$$
$\tau$ can be interpreted as the time at which the rocket vehicle composed of only fuel (no structure) will reach 0 mass. 


### Generalized Guidance Equation
It will be helpful to create a general formula for solving the guidance equations that will be derived. A guidance equation for generalized coordinate $q$ is defined:
$$\begin{align}
    \ddot q(t) & = c_1 p_1(t) + c_2 p_2(t) \\
\end{align}$$
where
$$\begin{align}
    p_1(t)&=a_T(t) \\
    p_2(t)&=(T-t)a_T(t)
\end{align}$$
and where T is the time of guidance termination and $a_T$ is acceleration due to rocket thrust. $a_T$ is written in the form 
$$\begin{align}
    a_T(t) = a_0 + a_1(T-t) + a_2(T-t)^2 + ... + a_n(T-t)^n \\
\end{align}$$
In this problem,  $p_1(t)$ and $p_2(t)$ are given, and the following boundary conditions are provided
$$\begin{align}
    q_0 & = q(t_0) \\
    \dot q_0 & = \dot q(t_0) \\
    q_D & = q(T) \\
    \dot q_D & = \dot q(T)
\end{align}$$
where $t_0$ is the current time. In this case, the unknowns are $c_1$ and $c_2$. 

Integrating the generalized guidance equation yields the equations of constraint
$$\begin{align}
    \dot q_D - \dot q_0 
        & = \int_{t_0}^T \ddot q(t) dt
        = c_1 \int_{t_0}^T p_1(t) dt + c_2 \int_{t_0}^T p_2(t) dt \\
    q_D - q_0 - \dot q(t_0)T_{go}
        & = \int_{t_0}^T \int_{t_0}^t \ddot q(s) ds \; dt
        = c_1 \int_{t_0}^T \int_{t_0}^t p_1(s) ds \; dt
            + c_2 \int_{t_0}^T \int_{t_0}^t p_2(s) ds \; dt
\end{align}$$
This can be represented by the matrix equation
$$\begin{align}
    \begin{bmatrix}
    \dot q_D - \dot q_0 \\
    q_D - (q_o + \dot q_o T_{go})
    \end{bmatrix}
    = F 
    \begin{bmatrix}
    c_1 \\
    c_2
    \end{bmatrix}
\end{align}$$
where the $F$ matrix is a $2 \times 2$ matrix composed of the following entries
$$\begin{align}
    f_{11} & = a_o T_{go} + a_1 T_{go}^2/2 + \dots + a_nT_{go}^{n+1}/(n+1)\\
    f_{12} & = a_o T_{go}^2/2 + a_1 T_{go}^3/3 + \dots + a_nT_{go}^{n+2}/(n+2)\\
    f_{21} & = f_{12} \\
    f_{22} & = a_o T_{go}^3/3 + a_1 T_{go}^4/4 + \dots + a_nT_{go}^{n+3}/(n+3)
\end{align}$$
$c_1$ and $c_2$ can be solved from the matrix equation by inverting the $F$ matrix. This solves the general guidance equation.

### Radial Guidance


### Yaw Guidance
### Time-to-Go
### Final Tangential Velocity
### Orbit Targeting
### Engine Property Estimator
## Reference

## Notes
I think $a_T$ is defined using a Taylor expansion since the integral for $a_T$ yields a logarithm, which may take 30 times the amount of time to multiply. I generally do not care about the performance here.

Need to figure out whether to use _o or _0.

## Equation Trashlands
\(a = b\)

$$\begin{align}
    f_{11} & =\int_{t_0}^T p_1(t) dt\\
    f_{12} & = \int_{t_0}^T p_2(t) dt\\
    f_{21} & = \int_{t_0}^T \int_{t_0}^t p_1(s) ds \; dt\\
    f_{22} & = \int_{t_0}^T \int_{t_0}^t p_2(s) ds \; dt
\end{align}$$