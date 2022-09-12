import sys
import pickle
import os
import multiprocessing
import numpy as np
import time

sys.path.append("../src")
from coop.obj_fcts import caprr_coopt_interface


robot = "acrobot"
filename = "heatmap_q11q22_"+robot+".pickle"

N_PROC = 3
evals = 100000
n = 32

torque_limit = 6.0
m1 = 0.608
m2 = 2.205058149850632754e-01
l1 = 2.003056968710561492e-01
l2 = 3.990621374058070936e-01
R_init = np.diag((0.62, 0.62))
Q_init = np.diag((1.0, 1.0, 0.99, 0.99))


save_dir = os.path.join("..", "results", "myresults","heatmaps",robot)
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
save_file = os.path.join(save_dir, filename)

if robot == "pendubot":
    tau_max = [torque_limit, 0.0]
elif robot == "acrobot":
    tau_max =[0.0, torque_limit]
design_params = {
    "m": [m1, m2],
    "l": [l1, l2],
    "lc": [l1, l2],
    "b": [0.0, 0.0],
    "fc": [0.0, 0.0],
    "g": 9.81,
    "I": [m1*l1**2, m2*l2**2],
    "tau_max": tau_max,
}

backend = "sos_con" # or "najafi"
caprr_roaEst = caprr_coopt_interface(
    design_params, Q_init, R_init, backend=backend,
    najafi_evals=evals, estimate_clbk=None, robot=robot)


def roa(roa_interface, Q, R, idx1, idx2, array_size, array):
    vol = roa_interface.lqr_param_opt_obj(Q, R)
    # np.savetxt(os.path.join(save_dir, "roa_" +
    #            str(idx1).zfill(2)+"_"+str(idx2).zfill(2)), [vol])
    array[idx1*array_size[1]+idx2] = vol

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

i = 0
init = time.time()
vol_array = multiprocessing.Array('d', [0]*len(Q11Vals)*len(Q22Vals))
for c in comp_list2:
    manager = multiprocessing.Manager()
    jobs = []
    for cc in c:
        Q = np.diag((cc[0], cc[1], Q_init[2, 2], Q_init[3, 3]))
        p = multiprocessing.Process(target=roa, args=(caprr_roaEst,
                                                      Q,
                                                      R_init,
                                                      cc[2],
                                                      cc[3],
                                                      [len(Q11Vals), len(Q22Vals)],
                                                      vol_array))
        jobs.append(p)
        p.start()
    for proc in jobs:
        proc.join()
        i = i+1
        print(f"{i} jobs done...")
end = time.time()

prob_vols = np.zeros((len(Q11Vals), len(Q22Vals)))
for idx1, q1 in enumerate(Q11Vals):
    for idx2, q2 in enumerate(Q22Vals):
        prob_vols[idx1][idx2] = vol_array[idx1*len(Q22Vals)+idx2]

results = {"prob_vols": prob_vols,
           "yticks": Q11Vals,
           "xticks": Q22Vals,
           "backend": backend,
           "robot": robot,
           "execution_time": end-init}

print("execution_time: ",end-init)

outfile = open(save_file, 'wb')
pickle.dump(results, outfile)
outfile.close()
