import math
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
import numpy as np

# data from c++ runs
sample_counts_by_dim = {
    0: {
        10: [25, 179, 1281, 9905, 76957, 589307],
        # 2: [37, 251, 2041, 16875, 142577, 1260103],
        2: [20,92,664,4222,27272,187980,1025648,7129226,],
        # 1: [49, 485, 5065, 55245, 594337, 6467039],
        1: [36, 256, 2584, 21114, 191916, 1652844, 15964928,],  # connectionR
        0.9: [57, 587, 6265, 69325, 784593, 9057095],
        0.75: [69, 799, 9697, 122767, 1505901, 19355217]
    },
    1: {
        10: [25, 113, 625, 2335, 20413, 77187],
        # 2: [37, 137, 1081, 5231, 35125, 178129],
        2: [20,58,312,1084,6392,21522,127344,456598,],
        # 1: [49, 283, 2593, 15217, 153373, 911312],
        1: [36,168,1368,5646,46896,261372,2153072,],  # connectionR
        0.9: [57, 331, 3025, 19409, 189945, 1257363],
        0.75: [69, 411, 5017, 32139, 367725, 2830221]
    },
    2: {
        10: [19, 113, 431, 2297, 9913, 50281],
        # 2: [19, 137, 685, 4007, 21911, 108129],
        # 1: [37, 283, 1735, 13189, 85079, 575619],
        1: [30,168,840,4366,26600,155408,946434,],  # connectionR
        0.9: [37, 331, 2231, 16639, 116873, 781715],
        0.75: [55, 411, 3481, 29131, 213179, 1770563],
        2: [12, 58, 190, 1124, 3542, 16314, 69444, 231282,]
    },
}

dists_by_dim = {
    0: {
        10:   [12.37, 107.92, 847.52,  7143.56,  58524.75,   462975.74],
        2:    [21.6,  152.8,  1384.0,  12166.0,  107765.2,   994271.6],
        1:    [24,    287.5,  3375.25, 40020.75, 451478.75,  5088422.0],
        0.9:  [29.09, 355.1,  4162.1,  49246.88, 585288.1,   7022861.1],
        0.75: [33.84, 476.46, 6448.5,  88171.92, 1123114.68, 14997856.32]
    },
    1: {
        10:   [12.37, 76.04,  406.93,  1595.16,  15784.16,  59844.93],
        2:    [21.6,  84.48,  753.6,   3968,     26451.2,   141759.26],
        1:    [24,    177,    1746,    11081.11, 117646.0,  723299.54],
        0.9:  [29.09, 205.14, 1973.53, 13939.56, 140119.7,  979760.1],
        0.75: [33.84, 238.46, 3391.2,  22734,    272055.24, 2236446.11]
    },
    2: {
        10:   [8.92,  76.04,  277.227, 1652.33,  7222.65,   38654.78],
        2:    [7.2,   84.48,  447.0,   2925.25,  16868.25,  83660.8],
        1:    [17.44, 177,    1134,    9679.5,   64117.59,  449342.0],
        0.9:  [15.61, 205.14, 1480.71, 12009.22, 87663.8,   596418.36],
        0.75: [27.94, 238.46, 2316.6,  21155.81, 157192.72, 1372398.72]
    },
}

reg_dists_by_dim = {
    0: {
        10:   [16.48, 134.55, 1021.21, 8294.39,   66426.95,   518248.78],
        2:    [26.61, 189.66, 1646.65, 14128.72,  122679.15,  1110541.79],
        1:    [32.25, 361.55, 4051.65, 46364.6,   512703.13,  5691491.88],
        0.9:  [38.31, 441.92, 5003.23, 57612.51,  670705.92,  7912838.51],
        0.75: [45.55, 597.37, 7747.81, 102586.36, 1287164.86, 16904238.06]
    },
    1: {
        10:   [16.48, 89.68,  494.36,  1902.39,  17770.47,  67438.91],
        2:    [26.61, 104.12, 884.36,  4493.17,  30164.81,  157675.61],
        1:    [32.25, 216.7,  2085.26, 12804.42, 132958.92, 805544.04],
        0.9:  [38.31, 252.37, 2384.05, 16219.52, 161473.08, 1101220.17],
        0.75: [45.55, 303.0,  4041.63, 26653.01, 313053.92, 2496198.11]
    },
    # 2: {
    #     10:   [12.23, 89.68,  338.67,  1652.33,  8375.1,    43746.4],
    #     2:    [11,    104.12, 542.35,  2925.25,  19025.4,   94373.98],
    #     1:    [23.93, 216.7,  1374.12, 9679.5,   73105.53,  504602.57],
    #     0.9:  [22.63, 252.37, 1780.56, 12009.22, 100171.24, 677453.46],
    #     0.75: [36.94, 303.0,  2782.48, 21155.81, 181186.06, 1546608.46]
    # },
    2: {
        10: [2.5851824532964174, 5.752974930172158, 12.135262567972859, 40.144754875643436,
             70.4915740746966, 302.41413775735066, ],
        2: [6.34871236722407,27.87930538520697,88.02749965286473,271.8863101479761,
            949.8057403515081,4430.1794576872135,],
        1: [8.693332436601615,63.72846105339182,240.66918939795963,1365.5239793459887,
            5951.62592309318,31087.659627583347,],
        0.9: [8.224405358688948,60.29088389416369,338.01574167214,1475.0488432992574,
              9263.171447829034,44055.38888085638,],
        0.75: [15.625173431356481,92.15818608882682,387.9660648397343,2509.3268795151175,
               15962.262173790245,95841.52513651173,],
    },
}


epsilon_vals = [10, 2, 1, 0.9, 0.75]
dims = [2, 3, 4, 5, 6, 7]

# init sample_by_epsilon
sample_counts_by_epsilon = {}
dists_by_epsilon = {}
reg_dists_by_epsilon = {}
for type in range(3):
    sample_counts_by_epsilon[type] = {}
    dists_by_epsilon[type] = {}
    reg_dists_by_epsilon[type] = {}
    for dim_i in range(6):
        sample_counts_by_epsilon[type][dims[dim_i]] = \
            [sample_counts_by_dim[type][epsilon][dim_i] for epsilon in epsilon_vals]
        dists_by_epsilon[type][dims[dim_i]] = \
            [dists_by_dim[type][epsilon][dim_i] for epsilon in epsilon_vals]
        reg_dists_by_epsilon[type][dims[dim_i]] = \
            [reg_dists_by_dim[type][epsilon][dim_i] for epsilon in epsilon_vals]


def SetPlotRC():
    #If fonttype = 1 doesn't work with LaTeX, try fonttype 42.
    plt.rc('pdf',fonttype = 42)
    plt.rc('ps',fonttype = 42)
    # plt.rc('eps',fonttype = 1)
    plt.rcParams['text.usetex'] = True

def ApplyFont(ax):

    ticks = ax.get_xticklabels() + ax.get_yticklabels()

    text_size = 14.0

    for t in ticks:
        t.set_fontname('Times New Roman')
        t.set_fontsize(text_size)

    txt = ax.get_xlabel()
    txt_obj = ax.set_xlabel(txt)
    txt_obj.set_fontname('Times New Roman')
    txt_obj.set_fontsize(text_size)

    txt = ax.get_ylabel()
    txt_obj = ax.set_ylabel(txt)
    txt_obj.set_fontname('Times New Roman')
    txt_obj.set_fontsize(text_size)

    txt = ax.get_title()
    txt_obj = ax.set_title(txt)
    txt_obj.set_fontname('Times New Roman')
    txt_obj.set_fontsize(text_size)


def theoretical_sample_count(d, delta, epsilon, type):
    # d = 6
    beta = (delta * epsilon) / math.sqrt(1 + epsilon * epsilon)
    r = (2 * (epsilon + 1) * delta) / math.sqrt(1 + epsilon * epsilon)
    theta = r / beta
    res = 0
    if type == 0:
        res = (1.0 / d) * (2.07 * theta) ** d + (theta * math.sqrt(d) / 2.0) ** (d - 2)
    elif type == 1:
        if d % 2 == 0:
            res = (1.0 / d) * (1.46 * theta) ** d + (theta * math.sqrt(d / 8.0)) ** (d - 2)
        else:
            res = (1.0 / d) * (1.46 * theta) ** d + (theta * math.sqrt((2 * d - 1) / 16.0)) ** (d - 2)
    elif type == 2:
        res = (1.0 / d) * (1.19 * theta) ** d + (theta * math.sqrt(d / 12.0)) ** (d - 2)
    return res


def theoretical_dist_sqrd_(d, delta, epsilon, type):
    # d = 6
    beta = (delta * epsilon) / math.sqrt(1 + epsilon * epsilon)
    r = (2 * (epsilon + 1) * delta) / math.sqrt(1 + epsilon * epsilon)
    theta = r / beta
    res = 0
    if type == 0:
        res = (beta / d) * ((theta * math.sqrt(d / 4.0)) + math.sqrt(d) - 0.5) ** (d - 1)
    elif type == 1:
        if d % 2 == 0:
            res = (beta / d) * ((theta * math.sqrt(d / 8.0)) + math.sqrt((9.0 * d) / 4) - 0.5) ** (d - 1)
        else:
            res = (beta / d) * ((theta * math.sqrt((2*d - 1) / 16.0)) + math.sqrt((9.0 * d) / 4) - 0.5) ** (d - 1)
    elif type == 2:
        res = (beta / d) * ((theta * math.sqrt(d / 12.0)) + math.sqrt(d) - 0.5) ** (d - 1)
    return res


def theoretical_dist_sqrd(d, delta, epsilon, type):
    # d = 6
    beta = (delta * epsilon) / math.sqrt(1 + epsilon * epsilon)
    r = (2 * (epsilon + 1) * delta) / math.sqrt(1 + epsilon * epsilon)
    theta = r / beta
    res = 0
    if type == 0:
        # res = (beta / math.sqrt(d)) * (2*math.pi*math.e*(theta + 1 - 1.0/(2*math.sqrt(d))))**d
        res = ((theta + 1) * math.sqrt(d) - 0.5) ** (d - 2)
        res *= 4*beta*beta
    elif type == 1:
        if d % 2 == 0:
            res = (beta / d) * ((theta * math.sqrt(d / 8.0)) + math.sqrt((9.0 * d) / 4) - 0.5) ** (d - 1)
        else:
            res = (beta / d) * ((theta * math.sqrt((2*d - 1) / 16.0)) + math.sqrt((9.0 * d) / 4) - 0.5) ** (d - 1)
    elif type == 2:
        res = (beta / d) * ((theta * math.sqrt(d / 12.0)) + math.sqrt(d) - 0.5) ** (d - 1)
    return res


def colors(type):
    if type == 0:
        return "blue"
    if type == 1:
        return "red"
    if type == 2:
        return "green"


def plot_sample_graph_absolute_by_epsilon():
    d = 6
    plt.figure(1)
    for type in range(3):
        # samples_theoretical = []
        # for i in range(len(epsilon_vals)):
        #     samples_theoretical.append(theoretical_sample_count(d, 0.25, epsilon_vals[i], type))
        #plot
        plt.plot(epsilon_vals, sample_counts_by_epsilon[type][d], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "$|\chi_\Lambda^{\delta,\epsilon}|$: sample set comparison for $d=6,\delta=0.25$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.4, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.16, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.03, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("$\epsilon$", fontsize=12)
    plt.ylabel("$|\chi_\Lambda^{\delta,\epsilon}|$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.17)  # Increase the left margin
    plt.yscale('log')
    plt.show()


def plot_sample_graph_absolute_by_dim():
    epsilon = 2
    plt.figure(2)
    for type in range(3):
        #plot
        # plt.plot(dims, sample_counts_by_dim[type][epsilon], marker='o', color=colors(type))
        plt.plot([2, 3, 4, 5, 6, 7, 8, 9], sample_counts_by_dim[type][epsilon], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "$|U^\Lambda|$: sample set comparison for $\delta=0.33, \epsilon=2$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.95, "$\mathbb{Z}^d$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.85, "$D_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.75, "$A_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("dimension", fontsize=12)
    plt.ylabel("$|U^\Lambda|$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.17)  # Increase the left margin
    plt.yscale('log')
    plt.show()


def plot_sample_graph_theoretical_by_epsilon():
    d = 6
    plt.figure(3)
    for type in range(3):
        samples_theoretical = []
        for i in range(len(epsilon_vals)):
            samples_theoretical.append(theoretical_sample_count(d, 0.25, epsilon_vals[i], type))
        #plot
        plt.plot(epsilon_vals, [(samples_theoretical[i] * 1.0)/sample_counts_by_epsilon[type][d][i] for i in range(5)], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "$|\chi_\Lambda^{\delta,\epsilon}|$: theoretical sample set comparison for $d=6,\delta=0.25$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.88, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.28, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.07, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("$\epsilon$", fontsize=12)
    plt.ylabel("$\\frac{theoretical}{practical}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.17)  # Increase the left margin
    plt.show()


def plot_sample_graph_theoretical_by_dim():
    epsilon = 0.75
    plt.figure(4)
    for type in range(3):
        samples_theoretical = []
        for dim in dims:
            samples_theoretical.append(theoretical_sample_count(dim, 0.25, 0.75, type))
        #plot
        plt.plot(dims, [(samples_theoretical[i] * 1.0)/sample_counts_by_dim[type][epsilon][i] for i in range(6)], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "$|\chi_\Lambda^{\delta,\epsilon}|$: theoretical sample set comparison for $\delta=0.25, \epsilon=0.75$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.43, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.22, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.03, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("dimension", fontsize=12)
    plt.ylabel("$\\frac{theoretical}{practical}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.17)  # Increase the left margin
    plt.show()


########################
# collision complexity #
########################

def plot_dists_graph_absolute_by_epsilon():
    d = 6
    plt.figure(1)
    dists_theoretical = []
    for epsilon in epsilon_vals:
        beta = (0.25 * epsilon) / math.sqrt(1 + epsilon * epsilon)
        radius_used = (2 * (epsilon + 1) * 0.25) / math.sqrt(1 + epsilon * epsilon)
        dists_theoretical.append(radius_used * theoretical_sample_count(d, 0.25, epsilon, 2))
    for type in range(3):
        #plot
        plt.plot(epsilon_vals, dists_theoretical, marker='o', color=colors(type))
        plt.plot(epsilon_vals, reg_dists_by_epsilon[type][d], linestyle='--', marker='o', color=colors(type))
        for i in range(5):
            print("dash_over_reg={}".format(dists_theoretical[i]/reg_dists_by_epsilon[type][d][i]))
    plt.text(0.5, 1.05, "CC$(\chi_\Lambda^{\delta,\epsilon})$: comparing absolute values for $d=6,\delta=0.25$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.4, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.16, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.03, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("$\epsilon$", fontsize=12)
    plt.ylabel("$CC(\chi_\Lambda^{\delta,\epsilon})$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.19)  # Increase the left margin
    plt.yscale('log')
    plt.show()


def plot_dists_graph_absolute_by_dim():
    epsilon = 0.75
    plt.figure(2)
    for type in range(3):
        #plot
        plt.plot(dims, dists_by_dim[type][epsilon], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "CC$(\chi_\Lambda^{\delta,\epsilon})$: comparing absolute values for $\delta=0.25, \epsilon=0.75$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.95, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.85, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.75, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("dimension", fontsize=12)
    plt.ylabel("$CC(\chi_\Lambda^{\delta,\epsilon})$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.19)  # Increase the left margin
    plt.yscale('log')
    plt.show()


def plot_dists_graph_theoretical_by_epsilon():
    d = 6
    plt.figure(3)
    for type in range(3):
        dists_theoretical = []
        for i in range(len(epsilon_vals)):
            dists_theoretical.append(theoretical_dist_sqrd(d, 0.25, epsilon_vals[i], type))
        #plot
        plt.plot(epsilon_vals, [(dists_theoretical[i] * 1.0)/dists_by_epsilon[type][d][i] for i in range(5)], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "CC$(\chi_\Lambda^{\delta,\epsilon})$: theoretical CC comparison for $d=6,\delta=0.25$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.94, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.38, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.28, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("$\epsilon$", fontsize=12)
    plt.ylabel("$\\frac{theoretical}{practical}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.21)  # Increase the left margin
    plt.show()


def plot_dists_graph_theoretical_by_dim():
    epsilon = 0.75
    plt.figure(4)
    for type in range(3):
        dists_theoretical = []
        for dim in dims:
            dists_theoretical.append(theoretical_dist_sqrd(dim, 0.25, 0.75, type))
        #plot
        plt.plot(dims, [(dists_theoretical[i] * 1.0)/dists_by_dim[type][epsilon][i] for i in range(6)], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "CC$(\chi_\Lambda^{\delta,\epsilon})$: theoretical CC comparison for $\delta=0.25, \epsilon=0.75$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.46, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.12, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.02, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("dimension", fontsize=12)
    plt.ylabel("$\\frac{theoretical}{practical}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.21)  # Increase the left margin
    plt.show()

def dists_vs_samples():
    epsilon = 0.75
    d=6
    plt.figure(5)
    for type in range(3):
        dists_theoretical = []
        for dim in dims:
            dists_theoretical.append(theoretical_dist_sqrd(dim, 0.25, 0.75, type))
        #plot
        plt.plot(epsilon_vals, [sample_counts_by_epsilon[type][d][i]/reg_dists_by_epsilon[type][d][i] for i in range(5)], marker='o', color=colors(type))
    plt.text(0.5, 1.05, "dists over samples with $d=6, \delta=0.25$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.46, "$\mathbb{Z}^6$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.12, "$D_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.02, "$A_6^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("epsilon", fontsize=12)
    plt.ylabel("$\\frac{|samples|}{\sum \|x\|}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.21)  # Increase the left margin
    plt.show()

def plot_success_rate():
    rates = [0, 2, 7, 18, 41, 62, 82, 94, 95, 96, 99, 100, 100]
    radius = [43.8, 44.8, 45.8, 46.8, 47.8, 48.8, 49.8, 50.8, 51.8, 52.8, 53.8, 54.8, 89.8]
    plt.figure(1)
    plt.plot(radius, rates, marker='o', color=colors(type))
    plt.text(0.5, 1.05, "comparing from $r_0$=deterministic to $r_1$=PRM*",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.02, 0.1, "$r_0$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.93, 0.87, "$r_1$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("radius", fontsize=12)
    plt.ylabel("success%", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.21)  # Increase the left margin
    plt.show()


def vol_n_ball(d: int) -> float:
    vol: float
    p = math.pi
    if d == 2:
        vol = p
    elif d == 3:
        vol = (4* p) / 3
    elif d == 4:
        vol = (1 * p**2) / 2
    elif d == 5:
        vol = (8 * p**2) / 15
    elif d == 6:
        vol = (1 * p**3) / 6
    elif d == 7:
        vol = (16 * p**3) / 105
    elif d == 8:
        vol = (1 * p**4) / 24
    elif d == 9:
        vol = (32 * p**4) / 945
    elif d == 10:
        vol = (1 * p**5) / 120
    elif d == 11:
        vol = (64 * p**5) / 10395
    elif d == 12:
        vol = (1 * p**6) / 720
    return vol

def plot_samples_upper_limit():
    SetPlotRC() # test
    fig = plt.figure()
    ax = fig.add_subplot()
    epsilon = 2
    # lambda expressions for the different terms
    f_zn = lambda d: math.sqrt(d) / 2.0
    f_dn = lambda d: math.sqrt(d / 8.0) if d % 2 == 0 else math.sqrt(2*d - 1.0) / 4.0
    f_an = lambda d: math.sqrt((d * (d + 2.0)) / (12.0 * (d + 1)))
    # unit_ball_vol = lambda d: (1.0 / math.sqrt(d * math.pi)) * math.pow((2*math.pi*math.e) / d, d/2.0)
    n_upper = lambda d,eps: vol_n_ball(d) * math.pow(2+(2.0/eps), d)
    # n_upper = lambda d,eps: unit_ball_vol(d) * math.pow(math.sqrt(1+eps**2) / (eps * 0.3), d)
    # calculate the upper limits
    dim_zn = [d for d in range(2, 10 + 1)]
    dim_dn = [d for d in range(2, 12 + 1)]
    dim_an = [d for d in range(2, 12 + 1)]
    zn_upper_theory = [math.pow(f_zn(d), d) * n_upper(d, epsilon) for d in dim_an]
    dn_upper_theory = [math.pow(f_dn(d), d) * n_upper(d, epsilon) * 2 for d in dim_dn]
    an_upper_theory = [math.pow(f_an(d), d) * n_upper(d, epsilon) * math.sqrt(d + 1) for d in dim_an]
    # zn_upper_real = [21, 93, 665, 4223, 23793, 153541, 1025649, 7129227, 47674841] # r = 1
    # dn_upper_real = [21, 59, 313, 1085, 6393, 21523, 127345, 362615, 2766461, 7987435]
    # an_upper_real = [13, 59, 191, 945, 3543, 12283, 69445, 231283, 977461, 4381505, 14783315,]
    # zn_upper_real = [9,27,89,573,2301,8893,33809,129303,765589,] # r = beta-ball radius, eps = 10
    # dn_upper_real = [9,15,49,83,681,1403,4017,12309,40549,]
    # an_upper_real = [7,15,31,93,169,647,2111,3531,12607,]
    # zn_upper_real = [9,33,137,893,3321,16859,69233,284055,1642109,] # r = beta-ball radius, eps = 5
    # dn_upper_real = [9,15,49,163,933,1977,6033,19701,115429,]
    # an_upper_real = [7,15,51,93,379,1221,5785,12413,35883,]
    an_upper_real = [13,51,161,495,1681,7411,24313,75203,298897,1404397,4419007,] # beta-ball, de=(1, 2)
    dn_upper_real = [13,51,169,525,2749,9161,59665,155831,1001173,2674523,17773561,] # beta-ball, de=(1, 2)
    zn_upper_real = [13,81,425,2463,12277,69779,469457,2634777,14763893,] # beta-ball, de=(1, 2)
    # zn_upper_real = [61,595,6577,72797,] # de = (1, 0.5)
    # dn_upper_real = [61,331,3336,19409,]
    # an_upper_real = [43,331,2231,17678,]
    # plot
    plt.figure(1)
    plt.plot(dim_an, zn_upper_theory, marker='o', linestyle='--', color='r')
    plt.plot(dim_dn, dn_upper_theory, marker='o', linestyle='--', color='g')
    plt.plot(dim_an, an_upper_theory, marker='o', linestyle='--', color='b')
    plt.plot(dim_zn, zn_upper_real, marker='o', color='r')
    plt.plot(dim_dn, dn_upper_real, marker='o', color='g')
    plt.plot(dim_an, an_upper_real, marker='o', color='b')
    # config axes
    plt.yscale('log')
    plt.xlabel("Dimension", fontsize=12)
    plt.ylabel("Sample complexity", fontsize=12, rotation=90, ha='center')
    # make a legend
    custom_lines = [
        Line2D([0], [0], color='r', lw=2, linestyle='--'),  # Dashed line
        Line2D([0], [0], color='r', lw=2, linestyle='-'),  # Solid line
        Line2D([0], [0], color='g', lw=2, linestyle='--'),  # Dashed line
        Line2D([0], [0], color='g', lw=2, linestyle='-'),  # Dashed line
        Line2D([0], [0], color='b', lw=2, linestyle='--'),  # Dashed line
        Line2D([0], [0], color='b', lw=2, linestyle='-'),  # Dashed line
    ]
    plt.legend(custom_lines,
               ['$Z_n$ theory', '$Z_n$ practice', '$D_n^*$ theory', '$D_n^*$ practice', '$A_n^*$ theory', '$A_n^*$ practice'],
               loc='upper left')
    # plot
    plt.tight_layout()
    ApplyFont(plt.gca()) # test
    fig.savefig('sample_complexity.pdf', format='pdf')
    # plt.show()

def plot_edges_upper_limit():
    SetPlotRC() # test
    fig = plt.figure()
    ax = fig.add_subplot()
    epsilon = 2
    # lambda expressions for the different terms
    f_zn = lambda d: math.sqrt(d) / 2.0
    f_dn = lambda d: math.sqrt(d / 8.0) if d % 2 == 0 else math.sqrt(2*d - 1.0) / 4.0
    f_an = lambda d: math.sqrt((d * (d + 2.0)) / (12.0 * (d + 1)))
    # unit_ball_vol = lambda d: (1.0 / math.sqrt(d * math.pi)) * math.pow((2*math.pi*math.e) / d, d/2.0)
    eta = lambda d: math.pow((d) / (1.0 + d), d)
    gamma = lambda d: 1.0 - (math.pow(eta(d), d+2) - eta(d))/(d*eta(d) - (d + 1.0))
    n_upper_pre = lambda d,eps: vol_n_ball(d) * math.pow(2+(2.0/eps), d)
    n_upper = lambda d,eps: 2 * (1.0 + (1.0 / eps)) * gamma(d) * n_upper_pre(d, epsilon)

    # calculate the upper limits
    dim_zn = [d for d in range(2, 10 + 1)]
    dim_dn = [d for d in range(2, 12 + 1)]
    dim_an = [d for d in range(2, 12 + 1)]

    zn_upper_theory = [f_zn(d) * math.pow(f_zn(d), d) * n_upper(d, epsilon) for d in dim_an]
    dn_upper_theory = [f_dn(d) * math.pow(f_dn(d), d) * n_upper(d, epsilon) * 2 for d in dim_dn]
    an_upper_theory = [f_an(d) * math.pow(f_an(d), d) * n_upper(d, epsilon) * math.sqrt(d + 1) for d in dim_an]
    # an_upper_real = [8.47, 44.61, 145.83, 799.35,3018.75, 10489.75, 63016.34, 207530.84, 893288.00,
    #                  4068195.14, 13615176.51,]
    # dn_upper_real = [15.00, 44.61, 250.85, 907.82, 5493.79, 18792.95, 112945.57, 317249.94, 2515779.66,
    #                  7170476.34]
    # zn_upper_real = [15.00, 67.29,540.55, 3580.96, 20255.98, 133734.83, 910940.19, 6455664.96, 43653894.86,]

    # zn_upper_real = [4.017494210333896,16.585696764680367,74.33723687039692,544.6767574934491,2001.5395121042118, # eps=5, beta-ball
    #                  10545.272189242189,43267.367658935465,177166.7909069333,1057235.4888307345,]
    # dn_upper_real = [4.017494210333896,6.803270591006504,24.104965262003393,94.49255649962574,568.8886480276984,
    #                  1197.7930195000374,3585.4841395172434,12410.982872676961,74864.81922769015,]
    # an_upper_real = [3.0571479921904077,6.803270591006504,27.506615068596012,47.473808515354904,221.49755280878978,
    #                  747.5807847460159,3773.4617904190923,7920.63141675296,23041.64688135928,]
    an_upper_real = [25.39484946889628,111.51722154082788,352.1099986114589,1087.5452405919043,3799.2229614060325, #de = (1,2), beta-ball
                     17720.717830748854,57900.95799335615,178228.80659969937,728080.2357208671,3527679.059365642,11023846.819977248,]
    dn_upper_real = [22.33435029680755,111.51722154082786,344.33156386282957,1144.6204318358318,6151.679461864939,
                     21248.80649430929,144048.01622600848,371569.1182970509,2457596.516649647,6498678.367577906,44313735.222271085,]
    zn_upper_real = [22.334350296807543,166.5696338753964,927.5740942616013,5611.168780505323,28054.19026814919,
                     162727.86598260727,1132797.1957066383,6400510.045346974,36016437.951159954,]
    # plot
    plt.figure(1)
    plt.plot(dim_an, zn_upper_theory, marker='o', linestyle='--', color='r')
    plt.plot(dim_dn, dn_upper_theory, marker='o', linestyle='--', color='g')
    plt.plot(dim_an, an_upper_theory, marker='o', linestyle='--', color='b')
    plt.plot(dim_zn, zn_upper_real, marker='o', color='r')
    plt.plot(dim_dn, dn_upper_real, marker='o', color='g')
    plt.plot(dim_an, an_upper_real, marker='o', color='b')
    # config axes
    plt.yscale('log')
    plt.xlabel("Dimension", fontsize=12)
    plt.ylabel("Collision complexity", fontsize=12, rotation=90, ha='center')
    # make a legend
    custom_lines = [
        Line2D([0], [0], color='r', lw=2, linestyle='--'),  # Dashed line
        Line2D([0], [0], color='r', lw=2, linestyle='-'),  # Solid line
        Line2D([0], [0], color='g', lw=2, linestyle='--'),  # Dashed line
        Line2D([0], [0], color='g', lw=2, linestyle='-'),  # Dashed line
        Line2D([0], [0], color='b', lw=2, linestyle='--'),  # Dashed line
        Line2D([0], [0], color='b', lw=2, linestyle='-'),  # Dashed line
    ]
    plt.legend(custom_lines,
               ['$Z_n$ theory', '$Z_n$ practice', '$D_n^*$ theory', '$D_n^*$ practice', '$A_n^*$ theory', '$A_n^*$ practice'],
               loc='upper left')
    # plot
    plt.tight_layout()
    ApplyFont(plt.gca()) # test
    fig.savefig('collision_complexity.pdf', format='pdf')
    # plt.show()


def plot_samples_upper_limit_comparative():
    fig = plt.figure()
    ax = fig.add_subplot()
    epsilon = 1.0
    delta = 0.2
    beta = (delta * epsilon) / math.sqrt(1 + epsilon**2)
    R = 2
    # lambda expressions for the different terms
    f_zn = lambda d: math.sqrt(d) / 2.0
    f_dn = lambda d: math.sqrt(d / 8.0) if d % 2 == 0 else math.sqrt(2*d - 1.0) / 4.0
    f_an = lambda d: math.sqrt((d * (d + 2.0)) / (12.0 * (d + 1)))
    unit_ball_vol = lambda d: 1.0 / math.sqrt(d * math.pi) * math.pow((2*math.pi*math.e) / d, d/2.0)
    n_upper = lambda d,eps: (unit_ball_vol(d) / math.sqrt(d)) * math.pow(R / beta, d)
    # calculate the upper limits
    dim = [d for d in range(2, 13)]
    zn_upper = [math.pow(f_zn(d), d) * n_upper(d, epsilon) for d in dim]
    dn_upper = [math.pow(f_dn(d), d) * n_upper(d, epsilon) for d in dim]
    an_upper = [math.pow(f_an(d), d) * n_upper(d, epsilon) for d in dim]



    # plot
    plt.figure(1)
    plt.plot(dim, zn_upper, marker='o', linestyle='-', color='r')
    plt.plot(dim, dn_upper, marker='o', linestyle='-', color='g')
    plt.plot(dim, an_upper, marker='o', linestyle='-', color='b')
    plt.yscale('log')
    plt.text(0.5, 1.05, "theoretical upper limites for the connection ball with $\epsilon=1$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.95, "$\mathbb{Z}^d$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.65, "$D_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.52, "$A_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("dimensions", fontsize=12)
    plt.ylabel("$n_{lattice}^{\delta,\epsilon}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.21)  # Increase the left margin
    plt.show()

    plt.figure(2)
    # real
    sample_count_thm = {
        0: zn_upper, 1: dn_upper, 2: an_upper
    }
    # 1.5
    sample_count_real = {
        0: [80,948,12576,162526,2229872,29645510,], 1: [80,530,6376,45338,562735,4054324,], 2: [60,530,4380,37730,315112,2638984,]
    }
    # 1
    sample_count_real = {
        0: [36,256,2584,21114,191916,1652844,15964928,], 1: [36,168,1368,5646,46896,261372,2153072,], 2: [30,168,840,4366,26600,155408,946434,]
    }
    # 5
    sample_count_real = {
        0: [877,35585,1524633,66959413,], 1: [877,19195,760969,18162731,], 2: [673,19195,544111,15484467,]
    }

    dim_real = list(range(len(sample_count_real[0])))
    rel_size_zn = [((sample_count_real[0][i] * 1.0) / sample_count_thm[0][i]) for i in range(len(dim_real))]
    rel_size_dn = [((sample_count_real[1][i] * 1.0) / sample_count_thm[1][i]) for i in range(len(dim_real))]
    rel_size_an = [((sample_count_real[2][i] * 1.0) / sample_count_thm[2][i]) for i in range(len(dim_real))]
    plt.plot(dim_real, rel_size_zn, marker='o', linestyle='-', color='r')
    plt.plot(dim_real, rel_size_dn, marker='o', linestyle='-', color='g')
    plt.plot(dim_real, rel_size_an, marker='o', linestyle='-', color='b')
    plt.text(0.5, 1.05, "theoretical upper limites for the $R=1.5,\epsilon=1$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.95, "$\mathbb{Z}^d$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.63, "$D_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.23, "$A_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("dimensions", fontsize=12)
    plt.ylabel("$\\frac{practical}{theoretical}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.21)  # Increase the left margin
    plt.show()

def plot_samples_upper_limit_comparative_dim_const():
    fig = plt.figure()
    ax = fig.add_subplot()
    delta = 0.3
    R = 2
    d = 5
    epsilon = 1
    beta = lambda eps: (delta * eps / math.sqrt(1 + eps**2))
    beta_del = lambda de: (de * epsilon / math.sqrt(1 + epsilon**2))
    # lambda expressions for the different terms
    f_zn = math.sqrt(d) / 2.0
    f_dn = math.sqrt(d / 8.0) if d % 2 == 0 else math.sqrt(2*d - 1.0) / 4.0
    f_an = math.sqrt((d * (d + 2.0)) / (12.0 * (d + 1)))
    unit_ball_vol = 1.0 / math.sqrt(d * math.pi) * math.pow((2*math.pi*math.e) / d, d/2.0)
    n_upper = lambda eps: unit_ball_vol * math.pow(R / beta(eps), d)
    n_upper_del = lambda de: unit_ball_vol * math.pow(R / beta_del(de), d)
    # calculate the upper limits
    eps_vals = [10, 2, 1, 0.9, 0.75, 0.65, 0.5, 0.4, 0.3, 0.2]
    eps_vals = [10, 2, 1, 0.9, 0.75, 0.65, 0.5, 0.4]
    delta_vals = [0.33, 0.3, 0.25, 0.2]
    zn_upper = [math.pow(f_zn, d) * n_upper(e) for e in eps_vals]
    dn_upper = [math.pow(f_dn, d) * n_upper(e) * 2 for e in eps_vals]
    an_upper = [math.pow(f_an, d) * n_upper(e) * math.sqrt(d+1) for e in eps_vals]
    # zn_upper = [math.pow(f_zn, d) * n_upper_del(de) for de in delta_vals]
    # dn_upper = [math.pow(f_dn, d) * n_upper_del(de) for de in delta_vals]
    # an_upper = [math.pow(f_an, d) * n_upper_del(de) for de in delta_vals]

    plt.figure(1)
    # real
    sample_count_thm = {
        0: zn_upper, 1: dn_upper, 2: an_upper
    }
    # R=1, d=5, eps varies
    sample_count_real = {
        0: [4223, 6393, 21115, 30315, 48285, 78717, 210057, 532509, ],
        1: [1085, 1885, 5647, 7327, 13977, 20529, 60971, 149533],
        2: [945, 1265, 4367, 6857, 11449, 18289, 49011, 123353, ]
    }
    #R=2, d=5, eps varies
    sample_count_real = {
        0: [126767, 210057, 695469, 909479, 1563321, 2542123, 6747225, 17091657, ],
        1: [35179, 60971, 188303, 247023, 428745, 687547, 1830151, 4680881],
        2: [29131, 49011, 156305, 210155, 364417, 574279, 1577183, 3942297, ]
    }
    # R=2, d=4, eps varies
    # sample_count_real = {
    #     0: [9697, 15457, 38249, 48945, 75297, 109505, 245345, 513241, 1430985, 6589945],
    #     1: [5017, 7729, 19057, 24121, 37249, 53953, 123817, 257233, 716281, 3289969],
    #     2: [3481, 5551, 14221, 17521, 27021, 39551, 86601, 182451, 514211, 2357681]
    # }
    # R=2, d=4, eps=0.4, delta varies
    # sample_count_real = {
    #     0: [350441, 513241, 1062353, 2587017],
    #     1: [175897, 257233, 532118, 1293697],
    #     2: [125141, 182451, 380371, 925155]
    # }


    rel_size_zn = [((sample_count_real[0][i] * 1.0) / sample_count_thm[0][i]) for i in range(len(eps_vals))]
    rel_size_dn = [((sample_count_real[1][i] * 1.0) / sample_count_thm[1][i]) for i in range(len(eps_vals))]
    rel_size_an = [((sample_count_real[2][i] * 1.0) / sample_count_thm[2][i]) for i in range(len(eps_vals))]
    # rel_size_zn = [((sample_count_real[0][i] * 1.0) / sample_count_thm[0][i]) for i in range(len(delta_vals))]
    # rel_size_dn = [((sample_count_real[1][i] * 1.0) / sample_count_thm[1][i]) for i in range(len(delta_vals))]
    # rel_size_an = [((sample_count_real[2][i] * 1.0) / sample_count_thm[2][i]) for i in range(len(delta_vals))]
    plt.plot(eps_vals, rel_size_zn, marker='o', linestyle='-', color='r')
    plt.plot(eps_vals, rel_size_dn, marker='o', linestyle='-', color='g')
    plt.plot(eps_vals, rel_size_an, marker='o', linestyle='-', color='b')
    print(rel_size_zn)
    print(rel_size_zn)
    # plt.plot(delta_vals, rel_size_zn, marker='o', linestyle='-', color='r')
    # plt.plot(delta_vals, rel_size_dn, marker='o', linestyle='-', color='g')
    # plt.plot(delta_vals, rel_size_an, marker='o', linestyle='-', color='b')
    plt.text(0.5, 1.05, "theoretical upper limites for the $R=2,\delta=0.3,d=5$",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.98, 0.95, "$\mathbb{Z}^d$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             , bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.63, "$D_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.text(0.98, 0.23, "$A_d^*$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("delta", fontsize=12)
    plt.ylabel("$\\frac{practical}{theoretical}$", fontsize=12, rotation=0, labelpad=4, ha='right')
    plt.subplots_adjust(left=0.21)  # Increase the left margin
    plt.show()

def improved_theoretical_bound():
    dims = list(range(2, 20 + 1))
    term = lambda d, x: 1 - (d * math.pow(x, d) - d * x) / (math.pow(d + 1, 2) * x - d * d - d)
    bounds = []
    for d in dims:
        x = math.pow(d / (d + 1.0), d)
        bounds.append(term(d, x))
    # plot
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.figure(1)
    plt.plot(dims, bounds, marker='o', linestyle='-', color='r')
    plt.text(0.5, 1.05, "Value of $\gamma$ for varying dimensions",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.xlabel("Dimension", fontsize=12)
    plt.ylabel("Coefficient $\gamma$", fontsize=12, rotation=90, labelpad=4, ha='center')
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    # plt.subplots_adjust(left=0.275)  # Increase the left margin
    fig.savefig('annuli_bound.eps', format='eps')
    plt.tight_layout()
    # plt.show()



def plot_theoretical_de_for_lower_limit():
    fig = plt.figure()
    ax = fig.add_subplot()
    # lambda expressions for the different terms
    f = lambda d: 1.0 / math.sqrt(36 * d * d - 1)
    d_vals1 = np.arange(0.2, 1, 0.1)
    d_vals2 = np.arange(1, 10, 1)
    # calculate the upper limits
    dim = [d for d in range(2, 13)]
    ep_upper1 = [f(d) for d in d_vals1]
    ep_upper2 = [f(d) for d in d_vals2]
    # plot
    plt.plot(d_vals1, ep_upper1, marker='o', linestyle='-', color='black')
    plt.plot(d_vals2, ep_upper2, marker='o', linestyle='-', color='black')
    plt.axvline(x=1.0/36, color='red', linestyle='--', linewidth=2, label='Limit Line')
    # plt.yscale('log')
    plt.text(0.5, 1.05, "depedence of $\epsilon$ on $\delta$ for a valid lower limit.",
             fontsize=11, ha='center', transform=plt.gca().transAxes)
    plt.text(0.07, 0.06, "$\delta=0.167$",
             fontsize=11, ha='left', transform=plt.gca().transAxes
             ,bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.2'))
    plt.xlabel("$\delta$", fontsize=12)
    plt.ylabel("$\epsilon$   ", fontsize=12, rotation=0, labelpad=4, ha='right')
    # plt.subplots_adjust(left=0.21)  # Increase the left margin

    plt.show()

def plot_general(X, Y, X_title, Y_title, general_title):
    fig = plt.figure()
    ax = fig.add_subplot()
    # plot
    plt.plot(X, Y, marker='o', linestyle='-', color='black')

    plt.text(0.5, 1.05, general_title,
             fontsize=11, ha='center', transform=plt.gca().transAxes)

    plt.xlabel(X_title, fontsize=12)
    plt.ylabel(Y_title, fontsize=12, rotation=90, labelpad=4, ha='right')

    plt.gca().invert_xaxis()

    plt.show()


# this function plots the final  sample complexity graphs for the lattice paper
def plot_experiment_graphs():
    # samples
    # plot_sample_graph_absolute_by_dim()
    # plot_sample_graph_absolute_by_epsilon()
    # plot_sample_graph_theoretical_by_dim()
    # plot_sample_graph_theoretical_by_epsilon()
    # dists
    # plot_dists_graph_absolute_by_dim()
    # plot_dists_graph_absolute_by_epsilon()
    # plot_dists_graph_theoretical_by_dim()
    # plot_dists_graph_theoretical_by_epsilon()
    # dists_vs_samples()
    pass


if __name__ == '__main__':
    # plot_experiment_graphs()
    # plot_samples_upper_limit()
    plot_edges_upper_limit()


    # eps = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0.9, 0.8, 0.7, 0.6, 0.5]
    # list275 = [715944,527196,820873,849762,865491,822047,951451,1063858,1554200,7864026,12743740,24288173,44108505,92858996,276750462,]
    # list4 = [1146473,1106126,1734145,1741154,1788525,-10000000,-1,267856,-10000000,1540081,2722996,4073701,6496439,12730215,34555503]
    # plot_general(eps,
    #              list275,
    #              "$\epsilon$", "A*-Runtime", "A* runtime as a function of $\epsilon$: Zigzag, $\delta=2.75$")
    #
    # plot_general(eps,
    #              list4,
    #              "$\epsilon$", "A*-Runtime", "A* runtime as a function of $\epsilon$: Zigzag $\delta=4$")
    #


    # plot_samples_upper_limit_comparative()
    # plot_samples_upper_limit_comparative_dim_const()
    # improved_theoretical_bound()
    # plot_theoretical_de_for_lower_limit()
    # plot_success_rate()