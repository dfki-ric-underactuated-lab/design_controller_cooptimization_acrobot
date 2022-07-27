import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("../src")
from coop.roa_calc import calc_roa


c_par = [1.0, 1.0, 1.0, 1.0, 1.0]
m_par = [0.63, 0.3, 0.2]

roa_backend = "najafi"
robot = "acrobot"
save_dir = "../results/myresults/roa_iter_analysis/"

evals = [10, 100, 1000, 10000, 100000, 1000000]
vols = []

reps = 2

for e in evals:
    print(e)
    rep_vols = []
    for r in range(reps):
        print(".")
        sd = os.path.join(save_dir, "iter"+str(e), str(r).zfill(2))

        vol, rho_f, S = calc_roa(c_par=c_par,
                                 m_par=m_par,
                                 roa_backend=roa_backend,
                                 najafi_evals=e,
                                 robot=robot,
                                 save_dir=sd,
                                 plots=False)
        rep_vols.append(vol)
    vols.append(np.mean(rep_vols))

np.savetxt(os.path.join(save_dir, "evaluations"), evals)
np.savetxt(os.path.join(save_dir, "iteration_data"), vols)

plt.plot(evals, vols)
plt.xlabel("Najafi Evaluations")
plt.ylabel("ROA Volume")
plt.savefig(os.path.join(save_dir, "roa_vs_najafievs"))
# plt.show()
plt.close()

plt.plot(evals, vols)
plt.xlabel("Najafi Evaluations")
plt.ylabel("ROA Volume")
plt.yscale("log")
plt.savefig(os.path.join(save_dir, "roa_vs_najafievs_log"))
plt.show()
plt.close()
