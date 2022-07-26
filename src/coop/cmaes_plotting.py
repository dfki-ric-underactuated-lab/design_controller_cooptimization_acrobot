import os
import numpy as np
import matplotlib.pyplot as plt


def plot_cma_results(data_path, sign=1., save_to=None, show=False):
    fit_path = os.path.join(data_path, "fit.dat")

    data = np.loadtxt(fit_path, skiprows=1)

    evaluations = data.T[1]
    best = data.T[4]*sign

    plt.plot(evaluations, best)
    plt.xlabel("Evaluations")
    plt.ylabel("ROA Volume")
    if not (save_to is None):
        plt.savefig(save_to)
    if show:
        plt.show()
    plt.close()


def plot_cma_altopt_results(data_path, save_to=None, show=False):
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
    fig = plt.figure(figsize=(16, 12))
    for i, s in enumerate(sequence):
        if s == "d":
            color = "red"
            label = "design opt"
        if s == "c":
            color = "blue"
            label = "lqr opt"
        if i > 0:
            evs = np.asarray(ev_lists[i]) + evs_sum  # add evs from previous opts
            x = np.insert(evs, 0, evs_sum)  # insert last data point
            y = np.insert(volume_lists[i], 0, volume_lists[i-1][-1])
            if s == "d" and not model_label_plotted:
                plt.plot(x, y, color=color, label=label)
                model_label_plotted = True
            elif s == "c" and not lqr_label_plotted:
                plt.plot(x, y, color=color, label=label)
                lqr_label_plotted = True
            else:
                plt.plot(x, y, color=color)
        else:
            plt.plot(ev_lists[i], volume_lists[i], "ro", color="black")
        evs_sum += ev_lists[i][-1]

    # plt.text(0., 3., "final parameters:\nModel parameters:\n"+
    #                    str(final_model_par)+
    #                    "\nLQR parameters:\n"+str(final_lqr_par),
    #                    fontsize=16)

    plt.xlabel("Evaluations", fontsize=24)
    plt.ylabel("ROA Volume", fontsize=24)
    plt.legend(loc="upper left", fontsize=24)
    if not (save_to is None):
        plt.savefig(save_to)
    if show:
        plt.show()
    plt.close()
