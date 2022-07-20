import sys
import os
from datetime import datetime

sys.path.append("../src")
from coop.roa_paropt import roa_alternate_opt


init_pars = [1., 1., 1., 1., 1., 0.63, 0.3, 0.2]
par_prefactors = [1., 1., 1., 1., 1.,
                  1., 1., 1.]
bounds = [[0.1, 10], [0.1, 10], [0.1, 1], [0.1, 1], [0.1, 1],
          [0.1, 1], [0.2, 0.4], [0.2, 0.4]]
maxfevals_per_opt = 500
opt_order = ["d", "c"]
roa_backend = "najafi"
najafi_evals = 100000
robot = "acrobot"
num_proc = 0


timestamp = datetime.today().strftime("%Y%m%d-%H%M%S")
save_dir = os.path.join("data", robot, "lqr", "roa_altopt", timestamp+"dc")

best_par = roa_alternate_opt(init_pars=init_pars,
                             par_prefactors=par_prefactors,
                             bounds=bounds,
                             maxfevals_per_opt=maxfevals_per_opt,
                             opt_order=opt_order,
                             roa_backend=roa_backend,
                             najafi_evals=najafi_evals,
                             robot=robot,
                             save_dir=save_dir,
                             num_proc=num_proc)
