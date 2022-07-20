import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("../src")
from coop.roa_calc import calc_roa


c_par = [0.52479041, 0.13940642, 0.10231669, 0.10100421, 0.99860526]
m_par = [0.1, 0.2, 0.4]

roa_backend = "prob"
robot = "acrobot"
save_dir = "data/roa_iter_analysis/"

evals = [10, 100, 1000]
vols = []

reps = 2

for e in evals:
    print(e)
    rep_vols = []
    for r in range(reps):
        print(".")
        sd = os.path.join(save_dir, "iter"+str(e), str(r).zfill(2))

        vol = calc_roa(c_par=c_par,
                       m_par=m_par,
                       roa_backend=roa_backend,
                       najafi_evals=e,
                       robot=robot,
                       save_dir=sd,
                       plots=False)
        rep_vols.append(vol)
    vols.append(np.mean(rep_vols))

plt.plot(evals, vols)
plt.xlabel("Najafi Evaluations")
plt.ylabel("ROA Volume")
plt.savefig(os.path.join(save_dir, "roa_vs_najafievs"))
plt.close()

plt.plot(evals, vols)
plt.xlabel("Najafi Evaluations")
plt.ylabel("ROA Volume")
plt.yscale("log")
plt.savefig(os.path.join(save_dir, "roa_vs_najafievs_log"))
plt.close()
