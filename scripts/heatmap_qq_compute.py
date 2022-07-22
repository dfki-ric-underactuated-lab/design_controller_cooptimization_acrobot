import sys
import pickle
import numpy as np
import os
import shutil
import multiprocessing

sys.path.append("../src")
from coop.obj_fcts import caprr_coopt_interface


filename = 'najafi_vols_qq.pickle'

N_PROC = 2
evals = 100000
n = 32

m1 = 0.608
m2 = 2.205058149850632754e-01
l1 = 2.003056968710561492e-01
l2 = 3.990621374058070936e-01


save_dir = os.path.join("..", "results", "myresults")
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
save_file = os.path.join(save_dir, filename)


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
R_init = np.diag((0.62, 0.62))
Q_init = np.diag((1.0, 1.0, 0.99, 0.99))

caprr_najafi = caprr_coopt_interface(
    design_params, Q_init, R_init, backend="najafi",
    najafi_evals=evals, estimate_clbk=None)


def roa(roa_interface, Q, R, idx1, idx2, save_dir):
    vol = roa_interface.lqr_param_opt_obj(Q, R)
    np.savetxt(os.path.join(save_dir, "roa_" +
               str(idx1).zfill(2)+"_"+str(idx2).zfill(2)), [vol])


data_dir = "parallel_data"
if not os.path.exists(data_dir):
    os.makedirs(data_dir)

Q11Vals = np.linspace(0.1, 10.0, n)
Q22Vals = np.linspace(0.1, 10.0, n)

comp_list = []
for idx1, q1 in enumerate(Q11Vals):
    for idx2, q2 in enumerate(Q22Vals):
        comp_list.append([q1, q2, idx1, idx2])

comp_list2 = []
cc = []
for i, c in enumerate(comp_list):
    if i > 0 and i % N_PROC == 0:
        comp_list2.append(cc)
        cc = []
    cc.append(c)
if len(cc) > 0:
    comp_list2.append(cc)

manager = multiprocessing.Manager()
jobs = []
for c in comp_list2:
    for cc in c:
        Q = np.diag((cc[0], cc[1], Q_init[2, 2], Q_init[3, 3]))
        p = multiprocessing.Process(target=roa, args=(caprr_najafi,
                                                      Q,
                                                      R_init,
                                                      cc[2],
                                                      cc[3],
                                                      data_dir))
        jobs.append(p)
        p.start()
    for proc in jobs:
        proc.join()

prob_vols = np.zeros((len(Q11Vals), len(Q22Vals)))
for idx1, q1 in enumerate(Q11Vals):
    for idx2, q2 in enumerate(Q22Vals):
        prob_vols[idx1][idx2] = -np.loadtxt(os.path.join(
            data_dir, "roa_"+str(idx1).zfill(2)+"_"+str(idx2).zfill(2)))

results = {"prob_vols": prob_vols,
           "yticks": Q11Vals,
           "xticks": Q22Vals}

outfile = open(save_file, 'wb')
# pickle.dump(prob_vols, outfile)
pickle.dump(results, outfile)
outfile.close()

shutil.rmtree(data_dir)
