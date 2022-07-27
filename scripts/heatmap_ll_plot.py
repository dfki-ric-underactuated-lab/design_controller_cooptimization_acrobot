import os
import numpy as np
import matplotlib.pyplot as plt
import pickle

load_dir = "../results/myresults"
load_filename = 'heatmap_l1l2.pickle'
load_file = os.path.join(load_dir, load_filename)

save_filename = "l1l2_heatmap.pdf"
save_file = os.path.join(load_dir, save_filename)


mark = [0.39, 0.205]

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
plt.text(mark[0]-0.01, mark[1]+0.01, "1", color="green")
plt.xlim(0.2, 0.4)
plt.xticks([0.2, 0.25, 0.3, 0.35, 0.4])
plt.xlabel("$l_2$ in $m$")
plt.ylabel("$l_1$ in $m$")
plt.savefig(save_file, bbox_inches="tight")
plt.show()
