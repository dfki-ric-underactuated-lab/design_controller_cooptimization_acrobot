import os
import numpy as np
import matplotlib.pyplot as plt
import pickle

load_dir = "../results/myresults"
load_filename = 'najafi_vols_qq.pickle'
load_file = os.path.join(load_dir, load_filename)

save_filename = "qq_heatmap.pdf"
save_file = os.path.join(load_dir, save_filename)

mark = [0.15, 2.08]

infile = open(load_file, 'rb')
results = pickle.load(infile, encoding='bytes')
prob_vols = -results["prob_vols"]
yticks = results["yticks"]
xticks = results["xticks"]

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.size": 26
})

fig = plt.figure(figsize=(8, 8))
# left right bottom top
plt.imshow(prob_vols, cmap='plasma', extent=[np.min(xticks), np.max(
    xticks), np.min(yticks), np.max(yticks)], origin="lower")
plt.colorbar(shrink=0.8)
plt.scatter(mark[0], mark[1], s=200, color="green")
plt.text(mark[0]+0.35, mark[1]+0.42, "2", color="green")
plt.xlim(0.1, 10)
plt.xticks([2, 4, 6, 8, 10])
plt.xlabel("$q_{22}$")
plt.ylabel("$q_{11}$")
plt.savefig(save_file, bbox_inches="tight")
plt.show()
