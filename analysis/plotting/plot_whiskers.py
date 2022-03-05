# plots a box and whiskers plot of s^pred - E[s^random]

from statistics import median, stdev
import allocation.assignment_util
import allocation.onehot_allocation
from matplotlib import pyplot as plt
import numpy as np


def plot_whiskers_3c3(score_data, R=-1):
    # get the difference between the ability-based score and the expected value for each team
    outperform_difference = []  # s^pred - E[s^random] for each team

    # result: best_value, pred_value, adjusted_pred_value, actual_value, worst_value, expected_value, [x for x in all_assignments]
    for result in score_data:  # pull each possible team, in the form of indexes instead of IDs
        outperform_difference.append(result[3] - sum(result[-1]) / len(result[-1]))

    N = len(outperform_difference)
    print("3c3 Outperformance Distribution", "%>0", len([x for x in outperform_difference if x > 0]) / len(outperform_difference), "Mean:", sum(outperform_difference) / len(outperform_difference), "Median:", median(outperform_difference), "StdDev:", stdev(outperform_difference))
    
    fig, ax = plt.subplots(1, 1)  # get the plot axis
    box = ax.boxplot(outperform_difference, vert=False, patch_artist=True, medianprops=dict(color="black"))
    box['boxes'][0].set(facecolor="violet")  # set the face color

    ax.get_yaxis().set_visible(False)  # hide the y axis labels
    
    #plt.title("Histogram of rankings of " + ("GENERATED USERS (R=" + str(R) if R != -1 else "") + " trait-based and random assignments, 3c3 [N=" + str(len(ranks_tb)) + "]")
    axis_fontsize = 15
    
    plt.xlabel("Performance difference $s^{pred} - E[s^{random}]$ for all teams", fontsize=axis_fontsize)
    plt.xlim([-2,2])
    #plt.legend()
