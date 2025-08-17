## 2. Simulation Objects

Simulation objects accept guidance objects and config objects during initialization.

All simulation objects objects use the state vector

$$\begin{align}
    \begin{bmatrix}\; x & y & z & \dot x & \dot y & \dot z & m \;\end{bmatrix}
\end{align}$$

Where $x$, $y$, $z$ are the components of the position vector $\vec r$ in the global inertial frame, and m is mass.

Length is in $m$, mass in $kg$, time in $s$.

### 2.1. IntegratorSim 

Uses Runge-Kutta 4 integrator. Rocket modelled as being solely under the forces of gravity and acceleration. No turning dynamics are modelled, acceleration direction is determined each instant by the guidance commands.

### 2.2. KRPCClient

Uses the KRPC library to connect to KSP. Currently only works for locally-hosted KRPC servers.

Uses KRPC's internal autopilot to control pitch and heading.