import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


class logger:
    def __init__(self):
        self.Q_log = []
        self.R_log = []
        self.m2_log = []
        self.l1_log = []
        self.l2_log = []
        self.vol_log = []

    def log_and_print_clbk(self, design_params, rho_f, vol, Q, R):
        print("design params: ")
        print(design_params)
        print("Q:")
        print(Q)
        print("R")
        print(R)
        print("rho final: "+str(rho_f))
        print("volume final: "+str(vol))
        print("")
        self.Q_log.append(Q)
        self.R_log.append(R)
        self.m2_log.append(design_params["m"][1])
        self.l1_log.append(design_params["l"][0])
        self.l2_log.append(design_params["l"][1])
        self.vol_log.append(vol)


cma_data_paths = ["../results/paper/cmaes/controller-first",
                  "../results/paper/cmaes/design-first"]
cma_simul_data_path = "../results/paper/cmaes/simultaneous"
nelder_design_first_data_path = '../results/paper/nelder-mead/design-first/2stage_design_first_logger.pickle'
nelder_controller_first_data_path = '../results/paper/nelder-mead/controller-first/2stage_lqr_first_logger.pickle'
save_to = "../results/paper/histories_combined.pdf"

cma_colors = ["#1900ff", "#b500a7"]
cma_symul_color = "#de0000"
nelder_colors = ["#008f1d", "#ff9305"]
line_width = 5

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.size": 26
})
fig = plt.figure(figsize=(12, 8))

# cma-es design-first and controller-first
for pn, data_path in enumerate(cma_data_paths):

    dirs = os.listdir(data_path)

    sequence = []

    ev_lists = []
    volume_lists = []

    for i, d in enumerate(sorted(dirs)):
        path = os.path.join(data_path, d)
        fit_path = os.path.join(path, "outcmaes", "fit.dat")

        if "design" in d or "lqr" in d:
            data = np.loadtxt(fit_path, skiprows=1)
            evaluations = data.T[1]
            best = data.T[4]

            ev_lists.append(evaluations)
            volume_lists.append(-1.*best)

            if "design" in d:
                sequence.append("d")
                # final_model_par = np.loadtxt(os.path.join(path, "model_par.csv"))
            elif "lqr" in d:
                sequence.append("c")
                # final_lqr_par = np.loadtxt(os.path.join(path, "controller_par.csv"))
        elif "init" in d:
            sequence.append("i")
            ev_lists.append(np.array([0]))
            volume_lists.append(np.array([np.loadtxt(os.path.join(path, "vol"))]))

    evs_sum = 0

    lqr_label_plotted = False
    model_label_plotted = False
    for i, s in enumerate(sequence):
        if s == "d":
            color = "red"
            ls = "dotted"
        if s == "c":
            color = "blue"
            ls = "--"
        if i > 0:
            evs = np.asarray(ev_lists[i]) + evs_sum  # add evs from previous opts
            x = np.insert(evs, 0, evs_sum)  # insert last data point
            y = np.insert(volume_lists[i], 0, volume_lists[i-1][-1])
            if s == "d" and not model_label_plotted:
                plt.plot(x, y, ls=ls, color=cma_colors[pn], lw=line_width)
                model_label_plotted = True
            elif s == "c" and not lqr_label_plotted:
                plt.plot(x, y, ls=ls, color=cma_colors[pn], lw=line_width)
                lqr_label_plotted = True
            else:
                plt.plot(x, y, color=color)
        else:
            plt.plot(ev_lists[i], volume_lists[i], "ro", color="black")
        evs_sum += ev_lists[i][-1]

# cma-es simultaneous
fit_path = os.path.join(cma_simul_data_path, "outcmaes", "fit.dat")

data = np.loadtxt(fit_path, skiprows=1)
evaluations = data.T[1]
best = data.T[4]*(-1)
plt.plot(evaluations, best, color=cma_symul_color, lw=line_width)


# Nelder-Mead
infile = open(nelder_design_first_data_path, 'rb')
l_design_first = pickle.load(infile, encoding='bytes')
infile.close()

infile = open(nelder_controller_first_data_path, 'rb')
l_lqr_first = pickle.load(infile, encoding='bytes')
infile.close()

design_first_vol = l_design_first.vol_log
lqr_first_vol = l_lqr_first.vol_log

design_first_filtered = []
lqr_first_filtered = []

for i in range(len(design_first_vol)-1):
    design_first_filtered.append(np.max(design_first_vol[0:i+1]))
    lqr_first_filtered.append(np.max(lqr_first_vol[0:i+1]))

idx1 = 78
idx2 = 85
plt.plot(np.arange(len(design_first_filtered[:idx1])), design_first_filtered[:idx1], ls="dotted", color=nelder_colors[1], lw=line_width)
plt.plot(np.arange(idx1, len(design_first_filtered)), design_first_filtered[idx1:], ls="--", color=nelder_colors[1], lw=line_width)

plt.plot(np.arange(len(lqr_first_filtered[:idx2])), lqr_first_filtered[:idx2], ls="--", color=nelder_colors[0], lw=line_width)
plt.plot(np.arange(idx2, len(lqr_first_filtered)), lqr_first_filtered[idx2:], ls="dotted", color=nelder_colors[0], lw=line_width)

# legend
c1_patch = mpatches.Patch(color=cma_colors[0], label='CMA-ES Controller first')
c2_patch = mpatches.Patch(color=cma_colors[1], label='CMA-ES Design first')
c3_patch = mpatches.Patch(color=cma_symul_color, label='CMA-ES Simultaneous')
c4_patch = mpatches.Patch(color=nelder_colors[0], label='Nelder-Mead Controller first')
c5_patch = mpatches.Patch(color=nelder_colors[1], label='Nelder-Mead Design first')

plt.xticks()
plt.yticks()
plt.ylim(0, 20)
plt.xlabel("Evaluations")
plt.ylabel("ROA Volume")
plt.legend(handles=[c1_patch, c2_patch, c3_patch, c4_patch, c5_patch], loc="best", fontsize=18)
plt.savefig(save_to, bbox_inches="tight")
plt.show()
plt.close()
