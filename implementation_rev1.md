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


## Appendix B: Abbreviated Derivation

The following is an abbreviated derivation that follows that of Cherry [1]?

The following is a derivation of an iterative method for guiding a single-stage rocket ascent vehicle. The derivation largely follows that of Cherry [1], with some modifications:
1. Estimation of the final circumferential velocity $v_{\theta D}$ using RK4 integration instead of a Taylor expansion
2. Targeting orbits based on the elements $$ instead of $$.

In broad terms, the guidance method is derived by 
1. Applying the linear tangent law [2] 
2. 

The derivation is performed without concern for the speed of calculation.

Cherry explicitly avoids invoking the calculus of variations when deriving the guidance laws.

## References
[1] G. W. Cherry, "A General, Explicit, Optimizing Guidance Law for Rocket-Propelled Spaceflight," in *Astrodynamics Guidance and Control Conference, August 24-26, 1964, Los Angeles, CA, USA* [Online]. Available: ARC, https://arc.aiaa.org/doi/10.2514/6.1964-638

TODO: Perhaps tone down the formal reference.