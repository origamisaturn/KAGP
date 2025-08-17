## TODO

- potentially add radial and yaw subscripts to appendix table
- look at self._last_time in guidance_components.py/EnginePropertyEstimator, might be a bug.
- add orbital elements to symbol list, deconflict all other uses of symbols that are orbital elements.
- add r and vectors
- make definition of $\hat y$ more prominent
- note that diagram only shows outputs used in get_command(), many more outputs may be exposed to the log.
- add example script for using log
- remember to go back and add width/height/alt for all images
- write a custom CSS class for centered figures.
- Consider making the subheadings in C.6 stand out more, like underlining or prepending it with "finding X."
- The big implementation page should be split up in the style of the kRPC documentation page, but idk if mkdocs can handle that nav page.
- Do NOT use subscript T for final time stuff, reserved for thrust $a_T(t)$.

Variables I have to take a look at,
$$\begin{gather}
v_{\theta t, \textrm{tgt}} \\\\
v_{\theta T} \\\\
\theta_T \textrm{ vs } \nu_T
\end{gather}$$

- make a note how $\theta$ replaces $\nu$ in the program
- Do I chose $r_T$ or $r_{T,\textrm{tgt}}$?
- Really should consider whether or not to use $r_T$ or $r(T)$, this is confusing.