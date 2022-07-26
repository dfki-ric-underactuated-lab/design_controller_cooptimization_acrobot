# Co-optimization of Acrobot Design and Controller for Increased Certifiable Stability

This repository contains the code, data and plots of the paper [Co-optimization of Acrobot Design and Controller for Increased Certifiable Stability](https://dfki-ric-underactuated-lab.github.io/design_controller_cooptimization_acrobot/). The paper was accepted at the [IROS 2022 Conference](https://iros2022.org/). A preprint is available on [Researchgate](https://www.researchgate.net/publication/362129400_Co-optimization_of_Acrobot_Design_and_Controller_for_Increased_Certifiable_Stability).

### Background
Recent research on control of walking robots focuses on the study of undercatuated systems, where taking into account the systems passive dynamics is required to achieve truly dynamic and energy efficient behaviours. Many approaches to controlling such systems incorporate offline trajectory optimization and stabilization using online optimal control. Furthermore, design optimization can be carried out to optimize the physical parameters such as masses and link lengths. In this work we propose a novel a novel approach to co-optimzation of design and control parameters. The goal of this optimization is maximizing the Region of attraction (ROA) of a desired state that is associated to a fixed point of the closed loop dynamics. For a first case study, we consider a scenario that involves stabilizing an acrobot at its unstable upright posture using infinite horizon LQR control.

We employ two black box optimization algorithms, namely CMA-ES and Nelder-Mead to maximize the volume of the ROA, which is estimated using a probabilistic method based on najafi. For this, we consider a quadratic Lyapunov function that is constructed using the optimal cost-to-go obtained by solving the LQR problem for the linearized dynamics around the upright pose of the acrobot. In order to reduce the dimensionality of the problem, we optimize over the physical design variables and the control variables separately.

### Content
The scripts for generating the data and plots can be found in the [scripts](scripts) folder. The data and plots published in the paper are stored at [results/paper](results/paper). The source code for the acrobot dynamics, ROA computation, optimization and plotting is in the [src/coop](src/coop) folder.

### Installation
The requirements for the code can be installed with

    pip install -r requirements.txt

### Plots
</div>
<div align="center">
<img width="201" src="results/paper/ellipse_comparison_with_acrobots.png">
<img width="225" src="results/paper/heatmaps/q11q22_heatmap.png">
<img width="276" src="results/paper/histories_combined.png">
</div>

### Citation
Maywald, Lasse & Wiebe, Felix & Kumar, Shivesh & Javadi, Mahdi & Kirchner, Frank. (2022). Co-optimization of Acrobot Design and Controller for Increased Certifiable Stability. 10.13140/RG.2.2.36436.07043.

    @misc{maywald2022,
    author = {Maywald, Lasse and Wiebe, Felix and Kumar, Shivesh and Javadi, Mahdi and Kirchner, Frank},
    year = {2022},
    month = {07},
    pages = {},
    title = {Co-optimization of Acrobot Design and Controller for Increased Certifiable Stability},
    doi = {10.13140/RG.2.2.36436.07043}
    }

</div>
<div align="center">
  <img src="docs/static/logo.svg" style="width:281px">
  <img src="docs/static/ulab.gif" style="width:225px">
  <img src="docs/static/MRock-Logo.png" style="width:198px">
  <img src="docs/static/KiMMI-Logo.png" style="width:60px">
</div>

