from matplotlib import pyplot as plt
import scipy.stats
import itertools
import numpy

# plots the histogram of team assignment scores
def plot_histogram(iteration_scores, R=-1, split=False):
    # format data into histogram distributions
    data = [[x[i] for x in iteration_scores] for i in range(len(iteration_scores[0])-1)]  # -1 to not consider the last (all possibilities)
    all_assignments = list(itertools.chain.from_iterable([x[-1] for x in iteration_scores]))

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

    plot = axes[0].hist(data[4], bins=bins, range=axis_range, alpha=alpha, color="red", label="Known Worst")  # histogram for worst scores
    plot = axes[1].hist(data[0], bins=bins, range=axis_range, alpha=alpha, color="green", label="Known Best")  # histogram for best scores
    plot = axes[2].hist(bins[:-1], bins, weights=counts, range=axis_range, alpha=alpha, color="orange", label="All Possibilities")
    plot = axes[3].hist(data[3], bins=bins, range=axis_range, alpha=alpha, color="blue", label="Trait-Based")  # histogram for predicted actual

    for ax in range(len(axes)):
        axis = axes[ax]
        axis.legend()
        if split:
            if ax == 3:  # place text on the trait-based plot
                axis.text(2.8, .5, "Percentile: " + str(round(tb_wins / (tb_wins + all_wins), 4)))

            if R != -1:
                axis.set_xlabel("[Assigning 3 Generated Users" + (", target r^2 of " + str(R) if R != -1 else "") + "]Team Assignment Scores of " + ("Known Worst" if ax == 0 else "Known Best" if ax == 1 else "All Possible" if ax == 2 else "Trait-Based") + " Assignment")  # set x label
            else:
                axis.set_xlabel("[Assigning 3 Generated Users" + (", target r^2 of " + str(R) if R != -1 else "") + "]Team Assignment Scores of " + ("Known Worst" if ax == 0 else "Known Best" if ax == 1 else "All Possible" if ax == 2 else "Trait-Based") + " Assignment, N=" + str(len(data[0])))  # set x label
            axis.set_xlim([0, 3])  # set locations of x ticks
            axis.set_ylabel("Number of Teams")  # set y label
        else:
            if ax > 0:  # only run for the first
                continue
            if R != -1:
                axis.set_title("[Assigning 3 Generated Users" + (", target r^2 of " + str(R) if R != -1 else "") + "] Histogram of Scores via Assignment Methods with all Traits Applied, N=" + str(len(data[0])))
            else:
                axis.set_title("[Assigning 3 Users] Histogram of Scores with all Traits Applied, N=" + str(len(data[0])))
            axis.set_xlabel("Team Assignment Score")  # set x label
            axis.set_xlim([0, 3])  # set locations of x ticks
            axis.set_ylabel("Number of Teams")  # set y label
            #axis.text(2.8, .5, str(round(tb_wins / (tb_wins + all_wins), 4)))
        
    return fig