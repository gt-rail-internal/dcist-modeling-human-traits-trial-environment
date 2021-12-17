# analyzes the data to determine the results

from statistics import median, stdev
import allocation.assignment_util
import allocation.onehot_allocation
from matplotlib import pyplot as plt

# calculate the pairwise ranking, plot distribution of performance difference between actual scores and trait based

def plot_ranking_3c3(score_data, R=-1):
    # define the trait and tasks
    traits=["ot", "sa", "ni"]
    tasks=["s1", "s2", "s3"]

    ranks_tb = []  # trait-based performance rank compared to all assignments

    # result: best_value, pred_value, adjusted_pred_value, actual_value, worst_value, expected_value, [x for x in all_assignments]
    for result in score_data:  # pull each possible team, in the form of indexes instead of IDs
        # determine the rank of the trait-based predictions
        rank = list(reversed(sorted([round(x, 5) for x in result[-1]]))).index(round(result[3], 5)) + 1
        #print("Rank", rank, "val", result[3], "options", sorted([round(x, 5) for x in result[-1]]))  # debug ranking
        ranks_tb.append(rank)

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

    print("3c3 Rank Distribution", r1, r2, r3, r4, r5, r6, "Mean:", sum(ranks_tb) / len(ranks_tb), "Median:", median(ranks_tb))
    
    #plt.hist(ranks_random, bins=range(0, 121 + 1, 1), alpha=0.7, color="orange", label="Random Assignment")
    fig, ax = plt.subplots(1, 1)  # get the plot axis
    plt.hist(ranks_tb, bins=range(-1, 8, 1), alpha=0.7, color="blue", label="Trait-Based Assignment")
    plt.plot([1, 7], [len(ranks_tb) / 6, len(ranks_tb) / 6], color="goldenrod")
    plt.title("Histogram of rankings of " + ("GENERATED USERS (R=" + str(R) if R != -1 else "") + " trait-based and random assignments, 3c3 [N=" + str(len(ranks_tb)) + "]")
    plt.ylabel("# of trait-based assignments")
    plt.xlabel("Rank of the assignment compared to all 6 possible assignments")
    plt.xlim([1,6.9])
    plt.legend()


