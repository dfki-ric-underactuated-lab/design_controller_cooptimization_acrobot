import sys
import os

sys.path.append("../src")
from coop.nelder_optimization import nelder_alternate_opt

init_pars = [1., 1., 1., 1., 1., 0.63, 0.3, 0.2]
bounds = [[0.1, 10], [0.1, 10], [0.1, 1], [0.1, 1], [0.1, 1],
          [0.1, 1], [0.2, 0.4], [0.2, 0.4]]
maxfevals_per_opt = 2
opt_order = ["c", "d"]
roa_backend = "najafi"
najafi_evals = 100
robot = "acrobot"


save_dir = os.path.join("../results", "myresults", "nelder", "controller_first")

best_par = nelder_alternate_opt(init_pars=init_pars,
                                bounds=bounds,
                                maxfevals_per_opt=maxfevals_per_opt,
                                opt_order=opt_order,
                                roa_backend=roa_backend,
                                najafi_evals=najafi_evals,
                                robot=robot,
                                save_dir=save_dir)
