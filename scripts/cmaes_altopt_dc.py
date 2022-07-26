import sys
import os

sys.path.append("../src")
from coop.cmaes_optimization import cmaes_alternate_opt


init_pars = [1., 1., 1., 1., 1., 0.63, 0.3, 0.2]
bounds = [[0.1, 10], [0.1, 10], [0.1, 1], [0.1, 1], [0.1, 1],
          [0.1, 1], [0.2, 0.4], [0.2, 0.4]]
maxfevals_per_opt = 500
opt_order = ["d", "c"]
roa_backend = "najafi"
najafi_evals = 100000
robot = "acrobot"
num_proc = 2
plots = True

save_dir = os.path.join("../results", "myresults", "cmaes", "design_first")

best_par = cmaes_alternate_opt(init_pars=init_pars,
                               bounds=bounds,
                               maxfevals_per_opt=maxfevals_per_opt,
                               opt_order=opt_order,
                               roa_backend=roa_backend,
                               najafi_evals=najafi_evals,
                               robot=robot,
                               save_dir=save_dir,
                               plots=plots,
                               num_proc=num_proc)
