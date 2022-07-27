# Scripts

With the scrips in this folder the results and plots from the paper can be
recreated. 

### cmaes\_altopt\_cd.py

This script performs the *controller first* optimization with cma-es (blue line
in optimization history plot (Fig. 3)).

### cmaes\_altopt\_dc.py

This script performs the *design first* optimization with cma-es (purple line
in optimization history plot (Fig. 3)).

### cmaes\_simulataneous.py

This script performs the *simultaneous* optimization with cma-es (red line in
optimization history plot (Fig. 3)).

### heatmap\_ll\_compute.py

This script computes the heatmap in the l1-l2-plane with the initial controller
parameters (Fig. 4a).

### heatmap\_ll\_plot.py

This script creates the heatmap plot from the data computed with
*heatmap\_ll\_compute.py*. (Fig. 4a).

### heatmap\_qq\_compute.py

This script computes the heatmap in the q11-q22-plane with the design
parameters found by cma-es (Fig. 4b).

### heatmap\_qq\_plot.py

This script creates the heatmap plot from the data computed with
*heatmap\_qq\_compute.py*. (Fig. 4b).

### lqr.py

This script simulates and animates the LQR controller for the acrobot.

### nelder\_altopt\_cd.py

This script performs the *controller first* optimization with Nelder-Mead
(green line in optimization history plot (Fig. 3)).

### nelder\_altopt\_dc.py

This script performs the *design first* optimization with Nelder-Mead (yellow
line in optimization history plot (Fig. 3)).

### plot\_history.py

This script created the history plot (Fig.3) from the data computed with

    - cmaes\_altopt\_cd.py
    - cmaes\_altopt\_dc.py
    - cmaes\_simulataneous.py
    - nelder\_altopt\_cd.py
    - nelder\_altopt\_dc.py

### roa\_iterations.py

This script computes the ROA volume with different numbers of najafi
iterations. Useful for determining the number of iterations, which is necessary
for convergence.

### verification.py

This script verifies the ROA ellipse by testing a grid of start states in the
(q1, qw, 0, 0)-plane with the najafi method. The plot shows green points when
the corresponding start state belongs to the ROA and red points if not. The
projected ellipse is plotted alongside these points.

### verification\_simulation.py

This scrips verifies the ROA ellipse by simuating the acrobot dynamics with
the LQR controller from starting states sampled from within the ellipse.
