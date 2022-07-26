import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import pickle
import multiprocessing

sys.path.append("../src")
from coop.ellipsoid import getEllipsePatch
from coop.check import lqr_endstate_check_epsilon

calculation = True
plotting = True
N_PROC = 2

data_path = "../results/paper/cmaes/design-first/"

filename = os.path.join(data_path, 'roa_verification_test.pickle')

dpar = [0.22050581, 0.2003057, 0.39906214]
cpar = [2.07884366, 0.15380351, 0.98670823, 0.99673571, 0.61940116]

q1_delta = 0.5
q2_delta = 1.5
n = 4

q1Vals = np.linspace(np.pi-q1_delta, np.pi+q1_delta, n)
q2Vals = np.linspace(-q2_delta, q2_delta, n)

rho = np.loadtxt(os.path.join(data_path, "rho"))
S = np.loadtxt(os.path.join(data_path, "Smatrix"))

if calculation:
    comp_list = []
    for idx1, q1 in enumerate(q1Vals):
        for idx2, q2 in enumerate(q2Vals):
            comp_list.append([idx1, idx2])

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
    suc_dict = manager.dict()
    for c in comp_list2:
        jobs = []
        for cc in c:
            p = multiprocessing.Process(target=lqr_endstate_check_epsilon,
                                        args=(dpar,
                                              cpar,
                                              [q1, q2, 0., 0.],
                                              suc_dict,
                                              cc[0],
                                              cc[1],
                                              0.02))
            jobs.append(p)
            p.start()
        for proc in jobs:
            proc.join()

    suc_grid = np.zeros((len(q1Vals), len(q2Vals)))
    for idx1, q1 in enumerate(q1Vals):
        for idx2, q2 in enumerate(q2Vals):
            suc_grid[idx1][idx2] = suc_dict[str(idx1).zfill(2)+"_"+str(idx2).zfill(2)]

    outfile = open(filename, 'wb')
    pickle.dump(suc_grid, outfile)
    outfile.close()

if plotting:
    infile = open(filename, 'rb')
    suc_grid = pickle.load(infile, encoding='bytes').astype(np.float)

    fig, ax = plt.subplots()

    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif"
    })
    for idx1, q1 in enumerate(q1Vals):
        for idx2, q2 in enumerate(q2Vals):
            if suc_grid[idx1][idx2]:
                color = "green"
            else:
                color = "red"
            plt.plot(q1, q2, "x", color=color)
    p = getEllipsePatch(np.pi, 0.0, 0, 1, rho, S)
    ax.add_patch(p)
    # l = np.max([p.width, p.height])
    ax.grid(True)
    plt.xlabel("$q_1$ in $rad$")
    plt.ylabel("$q_2$ in $rad$")
    plt.savefig(os.path.join(data_path, "verification_plot"))
    plt.show()
