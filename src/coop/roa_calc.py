import os
import numpy as np
import yaml

from .obj_fcts import caprr_coopt_interface


def calc_roa(c_par=[1., 1., 1., 1., 1.],
             m_par=[0.63, 0.3, 0.2],
             roa_backend="najafi",
             najafi_evals=1000,
             robot="acrobot",
             save_dir="data/",
             plots=False):

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    mass = [0.608, m_par[0]]
    length = [m_par[1], m_par[2]]
    com = [length[0], length[1]]
    damping = [0.0, 0.0]
    cfric = [0., 0.]
    gravity = 9.81
    inertia = [mass[0]*length[0]**2, mass[1]*length[1]**2]
    if robot == "acrobot":
        torque_limit = [0.0, 5.0]
    if robot == "pendubot":
        torque_limit = [5.0, 0.0]

    goal = [np.pi, 0, 0, 0]

    design_params = {"m": mass,
                     "l": length,
                     "lc": com,
                     "b": damping,
                     "fc": cfric,
                     "g": gravity,
                     "I": inertia,
                     "tau_max": torque_limit}

    Q = np.diag((c_par[0], c_par[1], c_par[2], c_par[3]))
    R = np.diag((c_par[4], c_par[4]))

    roa_calc = caprr_coopt_interface(design_params=design_params,
                                     Q=Q,
                                     R=R,
                                     backend=roa_backend,
                                     najafi_evals=najafi_evals,
                                     robot = robot)
    roa_calc._update_lqr(Q=Q, R=R)
    vol, rho_f, S = roa_calc._estimate()

    np.savetxt(os.path.join(save_dir, "rho"), [rho_f])
    np.savetxt(os.path.join(save_dir, "vol"), [vol])
    # np.savetxt(os.path.join(save_dir, "rhohist"), rhoHist)
    np.savetxt(os.path.join(save_dir, "Smatrix"), S)

    # if plots:
    #     plotEllipse(goal[0], goal[1], 0, 1, rho_f, S,
    #                 save_to=os.path.join(save_dir, "roaplot"),
    #                 show=False)

    np.savetxt(os.path.join(save_dir, "controller_par.csv"), c_par)
    np.savetxt(os.path.join(save_dir, "design_par.csv"), m_par)

    par_dict = {"mass1": mass[0],
                "mass2": float(mass[1]),
                "length1": float(length[0]),
                "length2": float(length[1]),
                "com1": float(com[0]),
                "com2": float(com[1]),
                "inertia1": float(inertia[0]),
                "inertia2": float(inertia[1]),
                "damping1": damping[0],
                "damping2": damping[1],
                "coulomb_friction1": cfric[0],
                "coulomb_friction2": cfric[1],
                "gravity": gravity,
                "torque_limit1": torque_limit[0],
                "torque_limit2": torque_limit[1],
                "goal_pos1": goal[0],
                "goal_pos2": goal[1],
                "goal_vel1": goal[2],
                "goal_vel2": goal[3],
                "Q1": float(c_par[0]),
                "Q2": float(c_par[1]),
                "Q3": float(c_par[2]),
                "Q4": float(c_par[3]),
                "R": float(c_par[4]),
                "roa_beackend": roa_backend,
                "najafi_evaluations": najafi_evals,
                }

    with open(os.path.join(save_dir, "roa_calc_parameters.yml"), 'w') as f:
        yaml.dump(par_dict, f)

    return vol, rho_f, S
