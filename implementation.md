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

It will be desirable to know the equation for $\ddot q(t)$. In this problem, $p_1(t)$ and $p_2(t)$ are given, and the following boundary conditions are provided

$$\begin{align}
    q_0 & = q(t_0) \\
    \dot q_0 & = \dot q(t_0) \\
    q_D & = q(T) \\
    \dot q_D & = \dot q(T)
\end{align}$$

where $t_0$ is the current time. The unknowns are $c_1$ and $c_2$. 

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

$$\begin{align}
    \tan \alpha = A + Bt \\
    \ddot r = a_T\sin\alpha - \mu/r^2 + v_{\theta}^2/r \\
        = a_T\sin\alpha + g_{eff} \\
    g_{eff} = -\mu/r^2 + v_{\theta}^2/r \\
    \sin \alpha = C + Dt - g_{eff} \\
    \ddot r = C a_T + D a_T t \\
    \ddot r = c_1p_1(t) = c_2p_2(t) \\
    p_1(t) = a_T \\
    p_2(t) = (T-t)a_T
\end{align}$$

Then use (A15 - A20).

NOTE: NEED to differentiate symbology.

### Yaw Guidance

$$\begin{align}
    \tan \alpha_y = A + Bt \\
    \ddot y = a_T \sin \alpha_y + \vec g \cdot \vec y \\
    \sin \alpha_y(t) = C+Dt - \vec g \cdot \vec y/a_T \\
    \ddot y = C a_T + D a_T t \\
    \ddot y = c_1p_1(t) + c_2p_2(t) \\
    p_1(t) = a_T \\
    p_2(t) = (T-t)a_T \\
    a_T(T) = a_T(T) + \dot a_T(T)(t-T) + \ddot a_T(T)(t-T)^2/2 + \dots \\
    p_1(t) = a_0 + a_1(T-t) + a_2(T-t)^2 + \dots \\
\end{align}$$

Then copy values for $a_i$ in (A17) to (A20).

### Time-to-Go

$$\begin{align}
    \dot v_\theta = a_T \cos \alpha - \frac{\dot r v_\theta}{r} \\
    \dot v_\theta = a_T - [(1 - \cos \alpha)a_T + \frac{\dot r v_\theta}{r}] \\
    a_L = (1 - \cos \alpha) a_T + \frac{\dot r v_\theta }{r} \\
    \dot v_\theta = a_T - a_L \\
    T_{go} = \tau _o \{ 1 - \exp[-(v_{\theta D} - v_{\theta o} + \Delta v_{\theta L})/v_e] \} \\
    \Delta v_{\theta L} = \int_{t_0}^T a_L(t) dt \\
    \Delta v_{\theta L, n+1} = v_{\theta D} - v_{\theta F, n} + \Delta v_{\theta L, n} \\
    | v_{\theta D } - v_{\theta F, n} | < \epsilon \\
    Q_{n+1} = \exp[-(v_{\theta D} - v_{\theta o}) / v_e] Q_n/H(T_n) \\
    Q_{n} = \exp(-\Delta v_{\theta L, n}/v_e) \\
    H(T_n) = H_{F, n} = \exp[-(v_{\theta F, n} - v_{\theta o})/v_e]
\end{align}$$

### Final Tangential Velocity

Spacecraft position at $\vec r$. Origin at body center. Vector of normal distance from target orbital plane $\vec y$, parallel to the angular momentum of the target orbit.

Plane control frame is centered at spacecraft. 
$$\begin{align}
    \hat i = \frac{\vec r}{|r|} \\
    \hat j = \frac{\hat y \times \hat i}{|\hat y \times \hat i|} \\
    \hat k = \hat i \times \hat j
\end{align}$$

TODO: need to add figures.

$\beta(t)$ is declination of spacecraft position $\vec r$ relative to target orbital plane. Solve for $\cos \beta(t)$, $\sin \beta(t)$, in terms of $r(t)$, $y(t)$.

$$\begin{align}
\cos \beta(t) = \frac{\sqrt{r(t)^2 - y(t)^2}}{r(t)} \\
\sin \beta(t) = \frac{y(t)}{r(t)}
\end{align}$$

$\hat y(t)$ in plane control frame is 

$$\begin{align}
\hat y(t) = \begin{bmatrix} \sin \beta(t) & 0 & \cos \beta(t)\end{bmatrix}
\end{align}$$

Find $\hat \theta$ with $\dot y(t)$ and $v_\theta$. $\vec v_\theta$ acts only in $\hat j$ and $\hat k$.

Find $\vec a_T$.

Find $\vec a_T \cdot \vec y$.

$$\begin{align}
    \vec g \cdot \hat y = -\frac{\mu y}{r^3} \\
    \vec a_T \cdot \hat y =  \ddot y(t) + \frac{\mu y}{r^3}
\end{align}$$

Find $\vec a_T \cdot \hat r$

$$\begin{align}
    \vec a_T \cdot \hat r =  \ddot r(t) - g_{eff}(t)
\end{align}$$

Find $\vec a_T$ in PCF.

$$\begin{align}
    \vec a_T \cdot \hat i = \vec a_T \cdot \hat r\\
    \vec a_T \cdot \hat j = \sqrt{a_T^2 - (\vec a_T \cdot \hat i)^2 - (\vec a_T \cdot \hat k)^2}\\
    \vec a_T \cdot \hat k = \frac{\vec a_T \cdot \hat y - (\vec a_T \cdot \hat r)(\hat y \cdot \hat i)}{\hat y \cdot \hat k}
\end{align}$$

$$\begin{align}
    \vec v(t) \cdot \hat i = \dot r \\
    \vec v(t) \cdot \hat j = \sqrt{v(t)^2 - (\vec v(t) \cdot \hat i)^2 - (\vec v(t) \cdot \hat k)^2}\\
    \vec v(t) \cdot \hat k = \frac{\vec v(t) \cdot \hat y - (\vec v(t) \cdot \hat r)(\hat y \cdot \hat i)}{\hat y \cdot \hat k}
\end{align}$$

$$\begin{align}
    \hat \theta(t) = \frac{\vec v_\theta(t)}{v_\theta(t)}
        = \frac{\begin{bmatrix} 0 & \vec v \cdot \hat j & \vec v \cdot \hat k \end{bmatrix}}
        {v_\theta}
\end{align}$$

### Orbit Targeting

Will not derive.

$$\begin{align}
    a = \frac{r_p + r_a}{2} \\
    e = 1 - \frac{r_p}{a} \\
    r = a \frac{(1-e^2)}{1 + e\cos(\theta)} \\
    h = \sqrt{r_p \mu (1+e)} \\
    v_r = \mu/h e \sin(\theta) \\
    v_\theta = \frac{h}{r}
\end{align}$$

$$\begin{align}
\end{align}$$

### Engine Property Estimator

No derivation.

$$\begin{align}
    a_T = \frac{v_e}{\tau - t}
\end{align}$$

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

<!-- ![svg-test](test.svg) -->
<img src="test.svg" alt="svg-test" style="width:400px;height:400px">
