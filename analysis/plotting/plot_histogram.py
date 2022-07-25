from matplotlib import pyplot as plt
import scipy.stats
import itertools
import numpy
import seaborn

# plots the histogram of team assignment scores
def plot_histogram(iteration_scores, R=-1, split=False):
    # format data into histogram distributions
    data = [[x[i] for x in iteration_scores] for i in range(len(iteration_scores[0])-1)]  # -1 to not consider the last (all possibilities)
    all_assignments = sorted(list(itertools.chain.from_iterable([x[-1] for x in iteration_scores])))
    data[3] = sorted(data[3])
    data[0] = sorted(data[0])
    data[4] = sorted(data[4])

    # as an aside, determine how trait-based compares to all-assignments within a 3-user group
    tb_wins = 0  # count of trait-based wins
    all_wins = 0  # count of all-assignment wins
    tie = 0  # count of ties
    todos = [x[-1] for x in iteration_scores]  # all possible assignments for each set of 3 users (6)
    for d in range(len(data[0])):  # for each set of 3 users
        tb = data[3][d]  # pull their trait-based score
        for all in todos[d]:  # for each possible score for that set
            if tb > all:  # if trait-based wins
                tb_wins += 1
            if tb == all:  # if they tie (should be >= num of 3 user sets, 3276)
                tie += 1
            if tb < all:  # if all-assignment wins
                all_wins += 1

    print("Winnings: TB", tb_wins, "ALL", all_wins, "TIE", tie, "%tile", round(tb_wins / (tb_wins + all_wins), 4))
    print("Averages: M_all", sum(all_assignments) / len(all_assignments), "M_tb", sum(data[3]) / len(data[3]))
    
    bins = 50
    background_alpha = .4
    alpha = 0.5
    axis_range = [0, 3]

    (counts, bins) = numpy.histogram(all_assignments, bins=bins)

    fig, axes = plt.subplots(nrows=4 if split else 1, ncols=1)
    if not isinstance(axes, numpy.ndarray):
        axes = [axes, axes, axes, axes]

    # normalize to make a probability density distribution
    # data_counts = [] * 5
    # for i in [0, 3, 4]:
    #     (counts, bins) = numpy.histogram(data[i], bins=bins)
    #     total_sum = numpy.sum(counts)
    #     counts = counts / total_sum
    #     data_counts[i] = counts
    #     print("total", total_sum, "counts", counts, "total data", sum(counts))
    
    n = data[3]


    #axes[0].hist(bins[:-1], bins, weights=counts, range=axis_range, alpha=alpha, color="orange", label="Random Assignment")
    seaborn.histplot(ax=axes[0], data=all_assignments, bins=bins, alpha=alpha, color="orange", linewidth=0, label="Random Assignment")
    #plot = axes[1].hist(data[4], bins=bins, range=axis_range, alpha=alpha, color="red", label="Observed Worst Assignment")  # histogram for worst scores
    seaborn.histplot(ax=axes[1], data=data[4], bins=bins, alpha=alpha, color="red", linewidth=0, edgecolor="red", label="Observed Worst Assignment")
    #plot = axes[2].hist(data[0], bins=bins, range=axis_range, alpha=alpha, color="green", label="Observed Best Assignment")  # histogram for best scores
    seaborn.histplot(ax=axes[2], data=data[0], bins=bins, alpha=alpha, color="green", linewidth=0, edgecolor="green", label="Observed Best Assignment")  # histogram for best scores
    #(n, bins, patches) = axes[3].hist(data[3], bins=bins, range=axis_range, alpha=alpha, color="blue", label="Individualized Role Assignment")  # histogram for predicted actual
    seaborn.histplot(ax=axes[3], data=data[3], bins=bins, alpha=alpha, color="blue", linewidth=0, edgecolor="blue", label="Individualized Role Assignment")  # histogram for predicted actual

    medians =   [all_assignments[len(all_assignments) // 2] if len(all_assignments) % 2 == 0 else (all_assignments[len(all_assignments) // 2] + all_assignments[len(all_assignments) // 2 + 1]) / 2,
                 data[4][len(data[4]) // 2] if len(data[4]) % 2 == 0 else (data[4][len(data[4]) // 2] + data[4][len(data[4]) // 2 + 1]) / 2,
                 data[0][len(data[0]) // 2] if len(data[0]) % 2 == 0 else (data[0][len(data[0]) // 2] + data[0][len(data[0]) // 2 + 1]) / 2,
                 data[3][len(data[3]) // 2] if len(data[3]) % 2 == 0 else (data[3][len(data[3]) // 2] + data[3][len(data[3]) // 2 + 1]) / 2]

    print(">>>", medians)


    total = sum(n)
    unit = 1 / total
    max_n = max(n)
    max_ny = max_n / 500
    max_ratio = max_n * unit
    max_y = max_ratio / max_ny
    #max_y = 0.136836
    print("total", total, "unit", unit, "max_n", max_n, "max_ratio", max_ratio, "max_y", max_y)

    ymax_normal = 548.10 # 500
    ymax_all = ymax_normal * (sum(counts) / len(data[0]))
    plot_labels = ["A", "B", "C", "D"]

    wilcox = scipy.stats.mannwhitneyu(data[3], all_assignments, alternative="greater")
    print("wilcox", wilcox)

    axis_fontsize = 15
    
    plt.xticks(fontsize=axis_fontsize-5)
    plt.yticks(fontsize=axis_fontsize-5)
    plt.legend(fontsize=axis_fontsize-5)

    for ax in range(len(axes)):
        axis = axes[ax]
        axis.spines['right'].set_visible(False)
        axis.spines['top'].set_visible(False)
        axis.legend(frameon=False, fontsize=axis_fontsize)
        median = medians[ax]
        if split:
            if ax == 3:  # place text on the trait-based plot
                #axis.text(2.8, .5, "Percentile: " + str(round(tb_wins / (tb_wins + all_wins), 4)))
                axis.set_xlabel("Cumulative Performance Score")

            if ax == 0:  # scale the y axis of the all assignments to maintain volume 
                axis.set_ylim([0, ymax_all])
                axis.set_yticks([600, 1800, 3000])
                axis.set_yticklabels([0.03, 0.09, .15])
                # plot the median
                median_x, median_y = [median, median], [0, 2400]
                axis.plot(median_x, median_y, color="grey", marker = '')
                axis.text(median + 0.05, 350 * 6, "median: " + str(round(median, 2)), fontsize=axis_fontsize-5)
                text_height = 400 * 6
            else:
                axis.set_ylim([0, ymax_normal])
                axis.set_yticks([100, 300, 500])
                axis.set_yticklabels([0.03, 0.09, .15])
                # plot the median
                median_x, median_y = [median, median], [0, 400]
                axis.plot(median_x, median_y, color="grey", marker = '')
                axis.text(median + 0.05, 350, "median: " + str(round(median, 2)), fontsize=axis_fontsize)
                text_height = 400

            if R != -1:
                axis.set_xlabel("[Assigning 3 Generated Users" + (", target r^2 of " + str(R) if R != -1 else "") + "]Team Assignment Scores of " + ("Known Worst" if ax == 0 else "Known Best" if ax == 1 else "All Possible" if ax == 2 else "Trait-Based") + " Assignment")  # set x label
            else:
                pass
                #axis.set_xlabel("[Assigning 3 Generated Users" + (", target r^2 of " + str(R) if R != -1 else "") + "]Team Assignment Scores of " + ("Known Worst" if ax == 0 else "Known Best" if ax == 1 else "All Possible" if ax == 2 else "Trait-Based") + " Assignment, N=" + str(len(data[0])))  # set x label
            axis.set_xlim([0, 3])  # set locations of x ticks
            axis.set_ylabel("Proportion of Teams", fontsize=axis_fontsize)  # set y label
            axis.text(0.1, text_height, plot_labels[ax], fontsize=25)
            

            
        else:
            if ax > 0:  # only run for the first
                continue
            if R != -1:
                axis.set_title("[Assigning 3 Generated Users" + (", target r^2 of " + str(R) if R != -1 else "") + "] Histogram of Scores via Assignment Methods with all Traits Applied, N=" + str(len(data[0])))
            else:
                axis.set_title("[Assigning 3 Users] Histogram of Scores with all Traits Applied, N=" + str(len(data[0])))
            axis.set_xlabel("Team Assignment Score")  # set x label
            axis.set_xlim([0.01, 3])  # set locations of x ticks
            axis.set_ylabel("Number of Teams")  # set y label
            #axis.text(2.8, .5, str(round(tb_wins / (tb_wins + all_wins), 4)))
        
    return fig

