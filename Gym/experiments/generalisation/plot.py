###########
# Script to plot the results of the experiments to test generalisation
# of the RL models
###########

from turtle import circle
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def plot():
    # LOAD DATA
    valid_maps = ["SOCHI", "circle", "squarish", "f"]
    obs_types = ["Frenet_trajectory", "original"]
    noises = [True, False]
    col_names = []
    for map_name in valid_maps:
        for obs_type in obs_types:
            for noise in noises:
                col_names.append("{}_{}_noise_{}".format(map_name, obs_type, noise))

    laptimes = pd.DataFrame(index=[a for a in range(5)], columns=col_names)
    completion = pd.DataFrame(index=[a for a in range(5)], columns=col_names)
    tot = {}
    for name in col_names:
        tot[name] = 0
        with open("./results/{}_results.txt".format(name), "r") as file:
            lines = file.readlines()
            for i in range(5):
                laptimes.loc[i, name] = float(lines[i + 1].strip().strip("[] "))
                completion.loc[i, name] = float(lines[i + 17].strip().strip("[] "))
                tot[name] += completion.loc[i, name]/laptimes.loc[i, name]

    # INIT PLOTS
    figure = plt.figure()

    ax_circle = figure.add_subplot(1, 3, 1)
    ax_squarish = figure.add_subplot(1, 3, 2)
    ax_f_shaped = figure.add_subplot(1, 3, 3)

    # PERSONALISED HISTOGRAM 
    ticklab = ["Ft w/ n", "Ft w/o n", "or w/ n", "or w/o n"]
    n_bins = 10
    circle_name_filter = lambda x: "circle" in x
    col_circ = [name for name in col_names if circle_name_filter(name)]
    data_circle = [np.array(completion[col], dtype=np.float32) for col in col_circ]
    hist_range = (0, 1)
    binned_data_circle = [np.histogram(d, range=hist_range, bins=n_bins)[0] for d in data_circle]
    binned_maximums = np.max(binned_data_circle, axis=1)
    x_locations = np.arange(0, np.max(binned_maximums)*len(binned_data_circle), np.max(binned_maximums))
    bin_edges = np.linspace(hist_range[0], hist_range[1], n_bins + 1)
    centers = 0.5 * (bin_edges + np.roll(bin_edges, 1))[1:]
    heights = np.diff(bin_edges)
    for x_loc, binned_data in zip(x_locations, binned_data_circle):
        lefts = x_loc - 0.5 * binned_data
        ax_circle.barh(centers, binned_data, height=heights, left=lefts)
    ax_circle.set_xticks(x_locations)
    ax_circle.set_xticklabels(ticklab)
    ax_circle.xaxis.grid(True)
    ax_circle.set_xlabel('circle')
    ax_circle.set_ylabel('percentage of completion of runs')
    #ax_circle.violinplot(data_circle, positions=x_locations, widths = 0.5*np.max(binned_maximums), showmeans=False)

    squarish_name_filter = lambda x: "squarish" in x
    col_square = [name for name in col_names if squarish_name_filter(name)]
    data_square = [np.array(completion[col], dtype=np.float32) for col in col_square]
    binned_data_square = [np.histogram(d, range=hist_range, bins=n_bins)[0] for d in data_square]
    binned_maximums_sq = np.max(binned_data_square, axis=1)
    x_locations_sq = np.arange(0, np.max(binned_maximums_sq)*len(binned_data_square), np.max(binned_maximums_sq))
    bin_edges_sq = np.linspace(hist_range[0], hist_range[1], n_bins + 1)
    centers_sq = 0.5 * (bin_edges_sq + np.roll(bin_edges_sq, 1))[1:]
    heights_sq = np.diff(bin_edges_sq)
    for x_loc, binned_data in zip(x_locations_sq, binned_data_square):
        lefts = x_loc - 0.5 * binned_data
        ax_squarish.barh(centers_sq, binned_data, height=heights_sq, left=lefts)
    ax_squarish.set_xticks(x_locations_sq)
    ax_squarish.set_xticklabels(ticklab)
    ax_squarish.xaxis.grid(True)
    ax_squarish.set_xlabel('squarish')
    ax_squarish.set_ylabel('percentage of completion of runs')
    #ax_squarish.violinplot(data_square, positions = x_locations_sq,  widths = 0.5*np.max(binned_maximums_sq), showmeans=False)




    f_name_filter = lambda x: "f_" in x
    col_f = [name for name in col_names if f_name_filter(name)]
    data_f = [np.array(completion[col], dtype=np.float32) for col in col_f]
    binned_data_f = [np.histogram(d, range=hist_range, bins=n_bins)[0] for d in data_f]
    binned_maximums_f = np.max(binned_data_f, axis=1)
    x_locations_f = np.arange(0, np.max(binned_maximums_f)*len(binned_data_f), np.max(binned_maximums_f))
    bin_edges_f = np.linspace(hist_range[0], hist_range[1], n_bins + 1)
    centers_f = 0.5 * (bin_edges_f + np.roll(bin_edges_f, 1))[1:]
    heights_f = np.diff(bin_edges_f)
    for x_loc, binned_data in zip(x_locations_f, binned_data_f):
        lefts = x_loc - 0.5 * binned_data
        ax_f_shaped.barh(centers_f, binned_data, height=heights_f, left=lefts)
    ax_f_shaped.set_xticks(x_locations_f)
    ax_f_shaped.set_xticklabels(ticklab)
    ax_f_shaped.xaxis.grid(True)
    ax_f_shaped.set_xlabel('f shaped')
    ax_f_shaped.set_ylabel('percentage of completion of runs')
    #ax_f_shaped.violinplot(data_f, positions= x_locations_f, widths = 0.5*np.max(binned_maximums_f), showmeans=False)

    plt.show()

if __name__ == "__main__":
    plot()
