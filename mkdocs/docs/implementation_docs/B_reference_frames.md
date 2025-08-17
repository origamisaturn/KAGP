## Appendix B: Reference Frames

### Global Frame

Axes are $\hat X$, $\hat Y$, $\hat Z$. Axes are inertial, origin is at center of the celestial body. In terms of (right ascension, declination), $\hat X$ is at $(0, 0)$, $\hat Y$ is at $(\frac{\pi}{2}, 0)$, and $\hat Z$ is at $(0, \frac{\pi}{2})$.

### Radial-Circumferential-Normal

$$\begin{align}
    \hat r &= \frac{\vec r}{r} \tag{B.1}\\\\
    \hat h &= \frac{\vec r \times \vec v}{|\vec r \times \vec v|} \tag{B.2}\\\\
    \hat \theta &= \hat h \times \hat r \tag{B.3} 
\end{align}$$

### Perifocal

Origin is at center of celestial body. $\hat p$ points to orbit periapsis, $\hat q$ on orbital plane at $\nu = \frac{\pi}{2}$, and $\hat w = \hat p \times \hat q$.

The following defines the perifocal axes in terms of the global frame.

$$\begin{align}
    \begin{bmatrix}
        & &\\\\
        \hat p & \hat q & \hat w\\\\
        & &
    \end{bmatrix}
    = R_z(\Omega) R_x(i) R_z(\omega)
\end{align}$$

### Plane Control

$$\begin{align}
    \hat i &= \frac{\vec r}{r} \tag{B.7}\\\\
    \hat j &= \frac{\hat y \times \hat i}{|\hat y \times \hat i|} \tag{B.8}\\\\
    \hat k &= \hat i \times \hat j \tag{B.9} 
\end{align}$$

Origin is at vehicle position $\vec r$. $\hat y$ is the normal vector of the target orbital plane, defined by $\Omega$ and $i$. Mainly used to simplify finding $v_\theta(T)$ and $\psi$.

### Topocentric

Origin is at vehicle position $\vec r$. $\hat n$ points North, $\hat e$ points East, $\hat d$ points downwards.

The following defines the topocentric axes in terms of the global frame.

$$\begin{align}
    \begin{bmatrix}
        & &\\\\
        \hat n & \hat e & \hat d\\\\
        & &
    \end{bmatrix}
    = R_z(RA) R_y(-DEC)
\end{align}$$

where $RA$ and $DEC$ are right ascension and declination, respectively.
