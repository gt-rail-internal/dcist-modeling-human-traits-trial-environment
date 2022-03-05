# analyzes the data to determine the results

from statistics import median, stdev
import allocation.assignment_util
import allocation.onehot_allocation
from matplotlib import pyplot as plt
import numpy as np


# calculate the pairwise ranking, plot distribution of performance difference between actual scores and trait based

def plot_ranking_3c3(score_data, R=-1):
    # define the trait and tasks
    traits=["ot", "sa", "ni"]
    tasks=["s1", "s2", "s3"]

    ranks_tb = []  # trait-based performance rank compared to all assignments
    outperform = 0  # num outperform expected value

    # result: best_value, pred_value, adjusted_pred_value, actual_value, worst_value, expected_value, [x for x in all_assignments]
    for result in score_data:  # pull each possible team, in the form of indexes instead of IDs
        # determine the rank of the trait-based predictions
        rank = list(reversed(sorted([round(x, 5) for x in result[-1]]))).index(round(result[3], 5)) + 1
        #print("Rank", rank, "val", result[3], "options", sorted([round(x, 5) for x in result[-1]]))  # debug ranking
        ranks_tb.append(rank)

        # determine if trait based outperformed average, if so increment the outperform count
        outperform = outperform + 1 if result[3] > sum(result[-1]) / len(result[-1]) else outperform

    r1 = 0
    r2 = 0 
    r3 = 0
    r4 = 0 
    r5 = 0
    r6 = 0

    for x in ranks_tb:
        r1 += 1 if x == 1 else 0
        r2 += 1 if x == 2 else 0
        r3 += 1 if x == 3 else 0
        r4 += 1 if x == 4 else 0
        r5 += 1 if x == 5 else 0
        r6 += 1 if x == 6 else 0

    N = len(ranks_tb)
    print("3c3 Rank Distribution", r1, "(" + str(r1 / N) + ")", r2, "(" + str(r2 / N) + ")", r3, "(" + str(r3 / N) + ")", r4, "(" + str(r4 / N) + ")", r5, "(" + str(r5 / N) + ")", r6, "(" + str(r6 / N) + ")", "Mean:", sum(ranks_tb) / len(ranks_tb), "Median:", median(ranks_tb), "Outperform:", outperform, "(" + str(outperform / N) + ")", "[" + str((r1 + r2 + r3)/N) + "]")
    
    #plt.hist(ranks_random, bins=range(0, 121 + 1, 1), alpha=0.7, color="orange", label="Random Assignment")
    fig, ax = plt.subplots(1, 1)  # get the plot axis
    #fig.get_axes()[0].get_legend().set_visible(False)  # remove the legend
    plt.hist(ranks_tb, bins=np.arange(-1, 8, 1)- 0.5, alpha=0.7, color="blue", edgecolor="black", label="Ability-Based Assignment")
    plt.plot([0, 7], [len(ranks_tb) / 6, len(ranks_tb) / 6], color="goldenrod", linewidth=5, label="Expected Value with Random Assignment")
    #plt.title("Histogram of rankings of " + ("GENERATED USERS (R=" + str(R) if R != -1 else "") + " trait-based and random assignments, 3c3 [N=" + str(len(ranks_tb)) + "]")
    axis_fontsize = 15
    plt.ylabel("# of teams", fontsize=axis_fontsize)
    plt.xlabel("Rank of ability-based assignment against all possible role assignments", fontsize=axis_fontsize)
    plt.xlim([0.5,6.5])
    plt.legend()




