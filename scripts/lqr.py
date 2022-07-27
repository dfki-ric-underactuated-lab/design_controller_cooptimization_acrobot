import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("../src")
from coop.acrobot.model.symbolic_plant import SymbolicDoublePendulum
from coop.acrobot.simulation.simulation import Simulator
from coop.acrobot.controller.lqr.lqr_controller import LQRController


# model parameters
mass = [0.608, 0.22050581]
length = [0.2003057, 0.39906214]
com = [length[0], length[1]]
damping = [0.0, 0.0]
cfric = [0., 0.]
gravity = 9.81
inertia = [mass[0]*length[0]**2., mass[1]*length[1]**2.]
torque_limit = [0.0, 5.0]


# simulation parameters
dt = 0.01
t_final = 5.0
integrator = "runge_kutta"
goal = [np.pi, 0., 0., 0.]
x0 = [np.pi+0.05, -0.2, 0.0, 0.0]

# lqr parameters
Q = np.diag([2.07884366, 0.15380351, 0.98670823, 0.99673571])
R = np.eye(2)*0.61940116

# simulation objects
plant = SymbolicDoublePendulum(mass=mass,
                               length=length,
                               com=com,
                               damping=damping,
                               coulomb_fric=cfric,
                               gravity=gravity,
                               inertia=inertia,
                               torque_limit=torque_limit)

sim = Simulator(plant=plant)

controller = LQRController(mass=mass,
                           length=length,
                           com=com,
                           damping=damping,
                           coulomb_fric=cfric,
                           gravity=gravity,
                           inertia=inertia,
                           torque_limit=torque_limit)
controller.set_goal(goal)
controller.set_cost_matrices(Q=Q, R=R)
controller.set_parameters(failure_value=0.0,
                          cost_to_go_cut=15)
controller.init()
T, X, U = sim.simulate_and_animate(t0=0.0, x0=x0,
                                   tf=t_final, dt=dt, controller=controller,
                                   integrator=integrator)

# plotting
fig, ax = plt.subplots(3, 1, figsize=(18, 9), sharex="all")
ax[0].plot(T, np.asarray(X).T[0], label="q1", color="blue")
ax[0].plot(T, np.asarray(X).T[1], label="q2", color="red")

ax[1].plot(T, np.asarray(X).T[2], label="q1 dot", color="blue")
ax[1].plot(T, np.asarray(X).T[3], label="q2 dot", color="red")

ax[2].plot(T, np.asarray(U).T[0], label="u1", color="blue")
ax[2].plot(T, np.asarray(U).T[1], label="u2", color="red")

plt.show()
