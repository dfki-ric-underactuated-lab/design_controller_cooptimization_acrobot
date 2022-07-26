import os
import pickle
import numpy as np
from scipy.optimize import minimize

from .obj_fcts import caprr_coopt_interface, logger
from .roa_calc import calc_roa


def nelder_controller_opt(
        model_pars=[0.63, 0.3, 0.2],
        init_pars=[1., 1., 1.],
        bounds=[[0, 1], [0, 1], [0, 1]],
        maxfevals=100,
        roa_backend="najafi",
        najafi_evals=10000,
        robot="acrobot",
        save_dir="data/"):

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    lqr_init_simp = [init_pars,
                     [1.5, 1, 1],
                     [1, 0.95, 1],
                     [1, 1, 0.95]]

    lqr_opts = {'maxiter': maxfevals,
                'disp': True,
                'return_all': True,
                'adaptive': True,
                'initial_simplex': lqr_init_simp}

    m1 = 0.608
    m2 = model_pars[0]
    l1 = model_pars[1]
    l2 = model_pars[2]

    design_params = {
        "m": [m1, m2],
        "l": [l1, l2],
        "lc": [l1, l2],
        "b": [0.0, 0.0],
        "fc": [0.0, 0.0],
        "g": 9.81,
        "I": [m1*l1**2, m2*l2**2],
        "tau_max": [0.0, 5.0],
    }

    Q = np.diag((1.0, 1.0, 1.0, 1.0))
    R = np.diag((1.0, 1.0))

    log = logger()
    caprr_najafi = caprr_coopt_interface(
                        design_params,
                        Q,
                        R,
                        backend=roa_backend,
                        najafi_evals=najafi_evals,
                        estimate_clbk=log.log_and_print_clbk)

    # this will use the initial simplex
    results = minimize(
            caprr_najafi.lqr_param_reduced_opt_obj,
            init_pars,
            bounds=bounds,
            method='nelder-mead',
            options=lqr_opts)

    filename = os.path.join(save_dir, '2stage_controller_first.pickle')
    outfile = open(filename, 'wb')
    pickle.dump(results, outfile)
    outfile.close()

    best_par = results["x"]

    # recalculate the roa for the best parameters and save plot
    _ = calc_roa(c_par=best_par,
                 m_par=model_pars,
                 roa_backend=roa_backend,
                 najafi_evals=najafi_evals,
                 robot=robot,
                 save_dir=os.path.join(save_dir, "final"),
                 plots=False)

    return best_par


def nelder_design_opt(
        lqr_pars=[1., 1., 1., 1., 1.],
        init_pars=[0.63, 0.3, 0.2],
        bounds=[[0, 1], [0, 1], [0, 1]],
        maxfevals=100,
        roa_backend="najafi",
        najafi_evals=10000,
        robot="acrobot",
        save_dir="data/"):

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    m1 = 0.608
    m2 = init_pars[0]
    l1 = init_pars[1]
    l2 = init_pars[2]

    design_params = {
        "m": [m1, m2],
        "l": [l1, l2],
        "lc": [l1, l2],
        "b": [0.0, 0.0],
        "fc": [0.0, 0.0],
        "g": 9.81,
        "I": [m1*l1**2, m2*l2**2],
        "tau_max": [0.0, 5.0],
    }

    Q = np.diag((lqr_pars[0], lqr_pars[1], lqr_pars[2], lqr_pars[3]))
    R = np.diag((lqr_pars[4], lqr_pars[4]))

    log = logger()
    caprr_najafi = caprr_coopt_interface(
                        design_params,
                        Q,
                        R,
                        backend=roa_backend,
                        najafi_evals=najafi_evals,
                        estimate_clbk=log.log_and_print_clbk)

    par_opts = {'maxiter': maxfevals,
                'disp': True,
                'return_all': True,
                'adaptive': True}

    results = minimize(
            caprr_najafi.design_opt_obj,
            init_pars,
            bounds=bounds,
            method='nelder-mead',
            options=par_opts)

    filename = os.path.join(save_dir, '2stage_design_first.pickle')
    outfile = open(filename, 'wb')
    pickle.dump(results, outfile)
    outfile.close()

    best_par = results["x"]

    # recalculate the roa for the best parameters and save plot
    _ = calc_roa(c_par=lqr_pars,
                 m_par=best_par,
                 roa_backend=roa_backend,
                 najafi_evals=najafi_evals,
                 robot=robot,
                 save_dir=os.path.join(save_dir, "final"),
                 plots=False)

    return best_par


def nelder_alternate_opt(init_pars=[1., 1., 1., 1., 1., 0.63, 0.3, 0.2],
                         bounds=[[0, 1], [0, 1], [0, 1], [0, 1], [0, 1],
                                 [0.3, 1], [0.3, 0.5], [0.5, 1.]],
                         maxfevals_per_opt=100,
                         opt_order=["d", "c"],
                         roa_backend="najafi",
                         najafi_evals=100000,
                         robot="acrobot",
                         save_dir="data/"):

    c_par = init_pars[:5]
    cc_par = [init_pars[0], init_pars[2], init_pars[4]]
    m_par = init_pars[5:]

    lqr_bounds = [bounds[0], bounds[2], bounds[4]]

    counter = 0
    _ = calc_roa(c_par=c_par,
                 m_par=m_par,
                 roa_backend=roa_backend,
                 robot=robot,
                 save_dir=os.path.join(save_dir, str(counter).zfill(2)+"_init"),
                 plots=False)

    counter += 1
    for o in opt_order:
        if o == "d":
            print("starting design optimization")
            #cc_par = [c_par[0], c_par[0], c_par[1], c_par[1], c_par[2]]
            m_par = nelder_design_opt(
                        lqr_pars=c_par,
                        init_pars=m_par,
                        bounds=bounds[5:],
                        maxfevals=maxfevals_per_opt,
                        roa_backend=roa_backend,
                        najafi_evals=najafi_evals,
                        robot=robot,
                        save_dir=os.path.join(save_dir, str(counter).zfill(2)+"_design"))

        elif o == "c":
            print("starting controller optimization")
            cc_par = nelder_controller_opt(
                        model_pars=m_par,
                        init_pars=cc_par,
                        bounds=lqr_bounds,
                        maxfevals=maxfevals_per_opt,
                        roa_backend=roa_backend,
                        najafi_evals=najafi_evals,
                        robot=robot,
                        save_dir=os.path.join(save_dir, str(counter).zfill(2)+"_lqr"))
            c_par = [cc_par[0], cc_par[0], cc_par[1], cc_par[1], cc_par[2]]
        counter += 1
    best_par = [c_par[0], c_par[1], c_par[2], c_par[3], c_par[4],
                m_par[0], m_par[1], m_par[2]]
    return best_par
