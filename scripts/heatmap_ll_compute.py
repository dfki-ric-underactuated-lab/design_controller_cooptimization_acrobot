import sys
import pickle
import numpy as np
import os
import shutil
import multiprocessing

sys.path.append("../src")
from coop.obj_fcts import caprr_coopt_interface


filename = 'najafi_vols_ll.pickle'

N_PROC = 2
evals = 100000
n = 32

m1 = 0.608
m2 = 0.2
l1 = 0.2
l2 = 0.4


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
R_init = np.diag((0.94, 0.94))
Q_init = np.diag((1.0, 1.0, 0.99, 0.99))

caprr_najafi = caprr_coopt_interface(
    design_params, Q_init, R_init, backend="najafi",
    najafi_evals=evals, estimate_clbk=None)


def roa(roa_interface, m2, l1, l2, idx1, idx2, array_size, array):
    vol = roa_interface.design_opt_obj([m2, l1, l2])
    # np.savetxt(os.path.join(save_dir, "roa_" +
    #            str(idx1).zfill(2)+"_"+str(idx2).zfill(2)), [vol])
    array[idx1*array_size[1]+idx2] = vol


data_dir = "parallel_data"
if not os.path.exists(data_dir):
    os.makedirs(data_dir)

l1Vals = np.linspace(0.2, 0.4, n)
l2Vals = np.linspace(0.2, 0.4, n)

comp_list = []
for idx1, l1 in enumerate(l1Vals):
    for idx2, l2 in enumerate(l2Vals):
        comp_list.append([l1, l2, idx1, idx2])

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
vol_array = multiprocessing.Array('d', [0]*len(l1Vals)*len(l2Vals))
for c in comp_list2:
    for cc in c:
        p = multiprocessing.Process(target=roa, args=(caprr_najafi,
                                                      m2,
                                                      cc[0],
                                                      cc[1],
                                                      cc[2],
                                                      cc[3],
                                                      [len(l1Vals), len(l2Vals)],
                                                      vol_array))
        jobs.append(p)
        p.start()
    for proc in jobs:
        proc.join()

prob_vols = np.zeros((len(l1Vals), len(l2Vals)))
for idx1, q1 in enumerate(l1Vals):
    for idx2, q2 in enumerate(l2Vals):
        #prob_vols[idx1][idx2] = -np.loadtxt(os.path.join(
        #    data_dir, "roa_"+str(idx1).zfill(2)+"_"+str(idx2).zfill(2)))
        prob_vols[idx1][idx2] = vol_array[idx1*len(l2Vals)+idx2]

results = {"prob_vols": prob_vols,
           "yticks": l1Vals,
           "xticks": l2Vals}

outfile = open(save_file, 'wb')
# pickle.dump(prob_vols, outfile)
pickle.dump(results, outfile)
outfile.close()

shutil.rmtree(data_dir)
