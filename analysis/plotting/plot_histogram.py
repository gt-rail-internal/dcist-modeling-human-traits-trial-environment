from matplotlib import pyplot as plt
import scipy.stats
import itertools
import numpy

# plots the histogram of team assignment scores
def plot_histogram(iteration_scores, task_noise=-1):
    # format data into histogram distributions
    data = [[x[i] for x in iteration_scores] for i in range(len(iteration_scores[0])-1)]  # -1 to not consider the last (all possibilities)
    all_assignments = list(itertools.chain.from_iterable([x[-1] for x in iteration_scores]))
    #print(len(data[-1]))

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
    fig, ax = plt.subplots(nrows=1, ncols=1)
    bins = 50
    background_alpha = .4
    alpha = 0.5
    axis_range = [0, 3]

    (counts, bins) = numpy.histogram(all_assignments, bins=bins)
    plot = ax.hist(data[4], bins=bins, range=axis_range, alpha=alpha, color="red", label="Known Worst")  # histogram for worst scores
    plot = ax.hist(data[0], bins=bins, range=axis_range, alpha=alpha, color="green", label="Known Best")  # histogram for best scores
    plot = ax.hist(bins[:-1], bins, weights=counts / 6, range=axis_range, alpha=alpha, color="orange", label="All Possibilities")
    plot = ax.hist(data[3], bins=bins, range=axis_range, alpha=alpha, color="blue", label="Trait-Based")  # histogram for predicted actual

    ax.legend()
    ax.set_title("[Assigning 3 Generated Users" + (", task noise of " + str(task_noise) if task_noise != -1 else "") + "] Histogram of Scores via Assignment Methods with all Traits Applied, N=" + str(len(data[0])))
    ax.set_xlabel("Team Assignment Score")  # set x label
    ax.set_xlim([0, 3])  # set locations of x ticks
    ax.set_ylabel("Number of Teams")  # set y label
    
    return fig