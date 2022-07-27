import sys
import os
import numpy as np

sys.path.append("../src")
from coop.ellipsoid import quadForm, sampleFromEllipsoid
from coop.acrobot.controller.lqr.lqr_controller import LQRController
from coop.acrobot.model.symbolic_plant import SymbolicDoublePendulum
from coop.check import lqr_check_epsilon


N = 1000

data_path = "../results/paper/cmaes/design-first"


rho = np.loadtxt(os.path.join(data_path, "rho"))
S = np.loadtxt(os.path.join(data_path, "Smatrix"))

dpar = np.loadtxt(os.path.join(data_path, "model_par.csv"))
cpar = np.loadtxt(os.path.join(data_path, "controller_par.csv"))

# model parameters
mass = [0.608, dpar[0]]
length = [dpar[1], dpar[2]]
com = [length[0], length[1]]
damping = [0.0, 0.0]
cfric = [0., 0.]
gravity = 9.81
inertia = [mass[0]*length[0]**2, mass[1]*length[1]**2]
torque_limit = [0.0, 5.0]

# simulation parameters
dt = 0.01
t_final = 5.0
goal = np.array([np.pi, 0., 0., 0.])

# controller parameters
Q = np.diag((cpar[0], cpar[1], cpar[2], cpar[3]))
R = np.diag((cpar[4], cpar[4]))

# simulaiton objects
plant = SymbolicDoublePendulum(mass=mass,
                               length=length,
                               com=com,
                               damping=damping,
                               gravity=gravity,
                               coulomb_fric=cfric,
                               inertia=inertia,
                               torque_limit=torque_limit)


controller = LQRController(mass=mass,
                           length=length,
                           com=com,
                           damping=damping,
                           gravity=gravity,
                           coulomb_fric=cfric,
                           inertia=inertia,
                           torque_limit=torque_limit)


controller.set_cost_matrices(Q, R)
controller.set_parameters(failure_value=0.0,
                          cost_to_go_cut=np.inf)
controller.init()

checker = lqr_check_epsilon(p=plant,
                            c=controller,
                            tf=t_final,
                            dt=dt,
                            goal=goal,
                            eps_p=0.1,
                            eps_v=0.2)

successes = []

for i in range(N):
    x = sampleFromEllipsoid(S, rho)
    x0 = goal + x
    suc = checker.sim_callback(x0)
    successes.append(suc)

n_suc = np.sum(successes)
print(f"Sampled {N} start states from the ROA ellipse")
print(f"{n_suc} / {N} forward simulations with LQR controller ended at the fixpoint.")
