import os
import time
import numpy as np
import yaml

from .obj_fcts import caprr_coopt_interface
from .optimizer import cma_par_optimization
from .roa_calc import calc_roa
from .loss_functions import (roa_lqrpar_lossfunc,
                             roa_modelpar_lossfunc,
                             roa_lqrandmodelpar_lossfunc)
from .cmaes_plotting import plot_cma_results, plot_cma_altopt_results
from .ellipsoid import plotEllipse


def cmaes_controller_opt(model_pars=[0.63, 0.3, 0.2],
                         init_pars=[1., 1., 1., 1., 1.],
                         bounds=[[0, 1], [0, 1], [0, 1], [0, 1], [0, 1]],
                         maxfevals=1000,
                         sigma0=0.4,
                         roa_backend="najafi",
                         najafi_evals=10000,
                         robot="acrobot",
                         save_dir="data/",
                         plots=False,
                         num_proc=0):

    mass = [0.608, model_pars[0]]
    length = [model_pars[1], model_pars[2]]
    com = [length[0], length[1]]
    damping = [0.0, 0.0]
    cfric = [0., 0.]
    gravity = 9.81
    inertia = [mass[0]*length[0]**2.0, mass[1]*length[1]**2.0]
    if robot == "acrobot":
        torque_limit = [0.0, 5.0]
    if robot == "pendubot":
        torque_limit = [5.0, 0.0]

    goal = [np.pi, 0, 0, 0]

    popsize_factor = 4

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # loss function setup
    loss_func = roa_lqrpar_lossfunc(bounds=bounds,
                                    roa_backend=roa_backend,
                                    najafi_evals=najafi_evals)
    loss_func.set_model_parameters(mass=mass,
                                   length=length,
                                   com=com,
                                   damping=damping,
                                   gravity=gravity,
                                   coulomb_fric=cfric,
                                   inertia=inertia,
                                   torque_limit=torque_limit)

    inits = loss_func.unscale_pars(init_pars)

    # optimization
    t0 = time.time()
    best_par = cma_par_optimization(
            loss_func=loss_func,
            init_pars=inits,
            bounds=[0, 1],
            save_dir=os.path.join(save_dir, "outcmaes"),
            sigma0=sigma0,
            popsize_factor=popsize_factor,
            maxfevals=maxfevals,
            num_proc=num_proc)
    opt_time = (time.time() - t0) / 3600  # time in h

    best_par = loss_func.rescale_pars(best_par)
    print(best_par)

    np.savetxt(os.path.join(save_dir, "controller_par.csv"), best_par)
    np.savetxt(os.path.join(save_dir, "time.txt"), [opt_time])

    par_dict = {"optimization": "lqr",
                "mass1": mass[0],
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
                "Q_init1": float(init_pars[0]),
                "Q_init2": float(init_pars[1]),
                "Q_init3": float(init_pars[2]),
                "Q_init4": float(init_pars[3]),
                "R_init": float(init_pars[4]),
                "popsize_factor": popsize_factor,
                "maxfevals": maxfevals,
                "roa_backend": roa_backend,
                "najafi_evals": najafi_evals,
                }

    with open(os.path.join(save_dir, "optimization_parameters.yml"), 'w') as f:
        yaml.dump(par_dict, f)

    # recalculate the roa for the best parameters and save plot
    vol, rho_f, S = calc_roa(c_par=best_par,
                             m_par=model_pars,
                             roa_backend=roa_backend,
                             najafi_evals=najafi_evals,
                             robot=robot,
                             save_dir=save_dir,
                             plots=plots)

    if plots:

        plot_cma_results(data_path=os.path.join(save_dir, "outcmaes"),
                         sign=-1.,
                         save_to=os.path.join(save_dir, "history"),
                         show=False)
        plotEllipse(goal[0], goal[1], 0, 1, rho_f, S,
                    save_to=os.path.join(save_dir, "roaplot"),
                    show=False)

    return best_par


def cmaes_design_opt(lqr_pars=[1., 1., 1., 1., 1.],
                     init_pars=[0.63, 0.3, 0.2],
                     bounds=[[0.3, 1], [0.3, 0.5], [0.5, 1.]],
                     maxfevals=1000,
                     sigma0=0.4,
                     roa_backend="najafi",
                     najafi_evals=100000,
                     robot="acrobot",
                     save_dir="data/",
                     plots=False,
                     num_proc=0):

    mass = [0.608, init_pars[0]]
    length = [init_pars[1], init_pars[2]]
    com = [length[0], length[1]]
    damping = [0.0, 0.0]
    cfric = [0., 0.]
    # damping = [0.081, 0.0]
    # cfric = [0.093, 0.186]
    gravity = 9.81
    inertia = [mass[0]*length[0]**2.0, mass[1]*length[1]**2.0]
    if robot == "acrobot":
        torque_limit = [0.0, 5.0]
    if robot == "pendubot":
        torque_limit = [5.0, 0.0]

    goal = [np.pi, 0, 0, 0]

    popsize_factor = 4

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # loss function setup
    loss_func = roa_modelpar_lossfunc(roa_backend=roa_backend,
                                      najafi_evals=najafi_evals,
                                      bounds=bounds)
    loss_func.set_model_parameters(mass=mass,
                                   length=length,
                                   com=com,
                                   damping=damping,
                                   gravity=gravity,
                                   coulomb_fric=cfric,
                                   inertia=inertia,
                                   torque_limit=torque_limit)

    Q = np.diag((lqr_pars[0], lqr_pars[1], lqr_pars[2], lqr_pars[3]))
    R = np.diag((lqr_pars[4], lqr_pars[4]))

    loss_func.set_cost_parameters(p1p1_cost=Q[0, 0],
                                  p2p2_cost=Q[1, 1],
                                  v1v1_cost=Q[2, 2],
                                  v2v2_cost=Q[3, 3],
                                  p1v1_cost=0.,
                                  p1v2_cost=0.,
                                  p2v1_cost=0.,
                                  p2v2_cost=0.,
                                  u1u1_cost=R[0, 0],
                                  u2u2_cost=R[1, 1],
                                  u1u2_cost=0.)

    # optimization
    t0 = time.time()
    best_par = cma_par_optimization(
            loss_func=loss_func,
            init_pars=init_pars,
            bounds=[0, 1],
            save_dir=os.path.join(save_dir, "outcmaes"),
            sigma0=sigma0,
            popsize_factor=popsize_factor,
            maxfevals=maxfevals,
            num_proc=num_proc)
    opt_time = (time.time() - t0) / 3600  # time in h

    best_par = loss_func.rescale_pars(best_par)
    print(best_par)

    np.savetxt(os.path.join(save_dir, "model_par.csv"), best_par)
    np.savetxt(os.path.join(save_dir, "time.txt"), [opt_time])

    par_dict = {"optimization": "design",
                "mass1": mass[0],
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
                "Q1": float(lqr_pars[0]),
                "Q2": float(lqr_pars[1]),
                "Q3": float(lqr_pars[2]),
                "Q4": float(lqr_pars[3]),
                "R": float(lqr_pars[4]),
                "init_pars": init_pars,
                "bounds": bounds,
                "popsize_factor": popsize_factor,
                "maxfevals": maxfevals,
                "roa_backend": roa_backend,
                "najafi_evals": najafi_evals,
                }

    with open(os.path.join(save_dir, "optimization_parameters.yml"), 'w') as f:
        yaml.dump(par_dict, f)

    # recalculate the roa for the best parameters and save plot
    vol, rho_f, S = calc_roa(c_par=lqr_pars,
                             m_par=best_par,
                             roa_backend=roa_backend,
                             najafi_evals=najafi_evals,
                             robot=robot,
                             save_dir=save_dir,
                             plots=plots)

    if plots:
        plot_cma_results(data_path=os.path.join(save_dir, "outcmaes"),
                         sign=-1.,
                         save_to=os.path.join(save_dir, "history"),
                         show=False)
        plotEllipse(goal[0], goal[1], 0, 1, rho_f, S,
                    save_to=os.path.join(save_dir, "roaplot"),
                    show=False)

    return best_par


def cmaes_alternate_opt(init_pars=[1., 1., 1., 1., 1., 0.63, 0.3, 0.2],
                        bounds=[[0, 1], [0, 1], [0, 1], [0, 1], [0, 1],
                                [0.3, 1], [0.3, 0.5], [0.5, 1.]],
                        maxfevals_per_opt=1000,
                        sigma_dec=1.0,
                        opt_order=["d", "c"],
                        roa_backend="najafi",
                        najafi_evals=100000,
                        robot="acrobot",
                        save_dir="data/",
                        plots=False,
                        num_proc=0):

    c_par = init_pars[:5]
    m_par = init_pars[5:]

    sigma_lqr = 0.4
    sigma_design = 0.4

    counter = 0
    _ = calc_roa(c_par=c_par,
                 m_par=m_par,
                 roa_backend=roa_backend,
                 robot=robot,
                 save_dir=os.path.join(save_dir, str(counter).zfill(2)+"_init"),
                 plots=plots)

    counter += 1
    for o in opt_order:
        if o == "d":
            print("starting design optimization")
            m_par = cmaes_design_opt(lqr_pars=c_par,
                                     init_pars=m_par,
                                     bounds=bounds[5:],
                                     maxfevals=maxfevals_per_opt,
                                     sigma0=sigma_design,
                                     roa_backend=roa_backend,
                                     najafi_evals=najafi_evals,
                                     robot=robot,
                                     save_dir=os.path.join(save_dir, str(counter).zfill(2)+"_design"),
                                     plots=plots,
                                     num_proc=num_proc)
            sigma_design *= sigma_dec
        elif o == "c":
            print("starting controller optimization")
            c_par = cmaes_controller_opt(model_pars=m_par,
                                         init_pars=c_par,
                                         bounds=bounds[:5],
                                         maxfevals=maxfevals_per_opt,
                                         sigma0=sigma_lqr,
                                         roa_backend=roa_backend,
                                         najafi_evals=najafi_evals,
                                         robot=robot,
                                         save_dir=os.path.join(save_dir, str(counter).zfill(2)+"_lqr"),
                                         plots=plots,
                                         num_proc=num_proc)
            sigma_lqr *= sigma_dec
        counter += 1
    best_par = [c_par[0], c_par[1], c_par[2], c_par[3], c_par[4],
                m_par[0], m_par[1], m_par[2]]

    # recalculate the roa for the best parameters and save plot
    vol, rho_f, S = calc_roa(c_par=c_par,
                             m_par=m_par,
                             roa_backend=roa_backend,
                             najafi_evals=najafi_evals,
                             robot=robot,
                             save_dir=save_dir,
                             plots=plots)

    if plots:
        plot_cma_results(data_path=os.path.join(save_dir, "outcmaes"),
                         sign=-1.,
                         save_to=os.path.join(save_dir, "history"),
                         show=False)
        goal = [np.pi, 0, 0, 0]
        plotEllipse(goal[0], goal[1], 0, 1, rho_f, S,
                    save_to=os.path.join(save_dir, "roaplot"),
                    show=False)
        plot_cma_altopt_results(data_path=save_dir,
                                save_to=os.path.join(save_dir, "history"),
                                show=False)

    return best_par


def cmaes_simul_opt(init_pars=[1., 1., 1., 1., 1., 0.63, 0.3, 0.2],
                    bounds=[[0, 1], [0, 1], [0, 1], [0, 1], [0, 1]],
                    maxfevals=1000,
                    sigma0=0.4,
                    roa_backend="najafi",
                    najafi_evals=100000,
                    robot="acrobot",
                    save_dir="data/",
                    plots=False,
                    num_proc=0):

    mass = [0.608, init_pars[5]]
    length = [init_pars[6], init_pars[7]]
    com = [0.3, 0.2]
    damping = [0.0, 0.0]
    cfric = [0., 0.]
    gravity = 9.81
    inertia = [mass[0]*length[0]**2., mass[1]*length[1]**2.]
    if robot == "acrobot":
        torque_limit = [0.0, 5.0]
    if robot == "pendubot":
        torque_limit = [5.0, 0.0]

    goal = [np.pi, 0, 0, 0]

    popsize_factor = 4

    # loss function setup
    loss_func = roa_lqrandmodelpar_lossfunc(roa_backend=roa_backend,
                                            najafi_evals=najafi_evals,
                                            bounds=bounds)
    loss_func.set_model_parameters(mass=mass,
                                   length=length,
                                   com=com,
                                   damping=damping,
                                   gravity=gravity,
                                   coulomb_fric=cfric,
                                   inertia=inertia,
                                   torque_limit=torque_limit)

    init_pars = loss_func.unscale_pars(init_pars)
    t0 = time.time()
    best_par = cma_par_optimization(loss_func=loss_func,
                                    init_pars=init_pars,
                                    bounds=[0, 1],
                                    sigma0=sigma0,
                                    save_dir=os.path.join(save_dir, "outcmaes"),
                                    popsize_factor=popsize_factor,
                                    maxfevals=maxfevals,
                                    num_proc=num_proc)
    opt_time = (time.time() - t0) / 3600  # time in h

    best_par = loss_func.rescale_pars(best_par)

    # np.savetxt(os.path.join(save_dir, "lqrandmodel_par.csv"), best_par)
    np.savetxt(os.path.join(save_dir, "time.txt"), [opt_time])

    par_dict = {"mass1": mass[0],
                "mass2": mass[1],
                "length1": length[0],
                "length2": length[1],
                "com1": com[0],
                "com2": com[1],
                "inertia1": inertia[0],
                "inertia2": inertia[1],
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
                "init_pars": list(init_pars),
                "bounds": list(bounds),
                "popsize_factor": popsize_factor,
                "maxfevals": maxfevals,
                }

    with open(os.path.join(save_dir, "optimization_parameters.yml"), 'w') as f:
        yaml.dump(par_dict, f)

    # recalculate the roa for the best parameters and save plot
    vol, rho_f, S = calc_roa(c_par=best_par[:5],
                             m_par=best_par[5:],
                             roa_backend=roa_backend,
                             najafi_evals=najafi_evals,
                             robot=robot,
                             save_dir=save_dir,
                             plots=plots)
    # best_Q = np.diag((best_par[0], best_par[1], best_par[2], best_par[3]))
    # best_R = np.diag((best_par[4], best_par[4]))

    # design_params = {"m": [mass[0], best_par[5]],
    #                  "l": [best_par[6], best_par[7]],
    #                  "lc": [best_par[6], best_par[7]],
    #                  "b": damping,
    #                  "fc": cfric,
    #                  "g": gravity,
    #                  "I": [mass[0]*best_par[6]**2, best_par[5]*best_par[7]**2],
    #                  "tau_max": torque_limit}

    # roa_calc = caprr_coopt_interface(design_params=design_params,
    #                                  Q=best_Q,
    #                                  R=best_R,
    #                                  backend=roa_backend)
    # roa_calc._update_lqr(Q=best_Q, R=best_R)
    # vol, rho_f, S = roa_calc._estimate()

    # np.savetxt(os.path.join(save_dir, "rho"), [rho_f])
    # np.savetxt(os.path.join(save_dir, "vol"), [vol])
    # # np.savetxt(os.path.join(save_dir, "rhohist"), rhoHist)
    # np.savetxt(os.path.join(save_dir, "Smatrix"), S)

    if plots:
        plot_cma_results(data_path=os.path.join(save_dir, "outcmaes"),
                         sign=-1.,
                         save_to=os.path.join(save_dir, "history"),
                         show=False)
        plotEllipse(goal[0], goal[1], 0, 1, rho_f, S,
                    save_to=os.path.join(save_dir, "roaplot"),
                    show=False)
    return best_par
