import sys
import os

sys.path.append("../src")
from coop.cmaes_optimization import cmaes_simul_opt


init_pars = [1., 1., 1., 1., 1., 0.63, 0.3, 0.2]
bounds = [[0.1, 10], [0.1, 10], [0.1, 1], [0.1, 1], [0.1, 1],
          [0.1, 1], [0.2, 0.4], [0.2, 0.4]]
maxfevals = 1000
roa_backend = "najafi"
najafi_evals = 100000
robot = "acrobot"
num_proc = 2
plots = True

save_dir = os.path.join("../results", "myresults", "cmaes", "simultaneous")


best_par = cmaes_simul_opt(init_pars=init_pars,
                           bounds=bounds,
                           maxfevals=maxfevals,
                           roa_backend=roa_backend,
                           najafi_evals=najafi_evals,
                           robot=robot,
                           save_dir=save_dir,
                           plots=plots,
                           num_proc=num_proc)
