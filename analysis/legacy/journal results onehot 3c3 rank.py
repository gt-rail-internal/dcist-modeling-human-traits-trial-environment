# analyzes the data to determine the results

from statistics import median, stdev
import allocation.assignment_util
import allocation.onehot_allocation

# calculate the pairwise ranking, plot distribution of performance difference between actual scores and trait based

import sys, os.path, pickle

# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]
prediction_tasks = ["s1", "s2", "s3"]

import os, random

# for N iterations, choose 3 users, get best predicted team of 3, compare to actual team of 3
N = 10000

used_samplings = []  # list of 6-user samples we have already looked at

# pull the user scores from the logs (or generate them randomly)
#user_scores = allocation.assignment_util.process_logs("logs")  # all user scores (even those who failed to have complete data)
user_scores = allocation.assignment_util.generate_fake_user_scores(N=30, trait_noise=0.8, task_noise=1)[0]

# pull out the complete users (applicable for a 6-user subset)
complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data
complete_user_scores_ids = list(complete_user_scores.keys())  # pull the IDs of the users who completed all stages

ranks_tb = []  # holds the data for each team combination

for ids in allocation.assignment_util.pullSubsets(len(complete_user_scores_ids), len(traits)):  # pull each possible team, in the form of indexes instead of IDs
    test_ids = [complete_user_scores_ids[x] for x in ids]
    # train and evaluate
    # extract the test/train user IDs from the team indexes
    result = allocation.onehot_allocation.onehot_allocation(user_scores, test_ids, traits, tasks)  # best_value, pred_value, adjusted_pred_value, actual_value, worst_value, expected_value, [x for x in all_assignments]
    # determine the rank of the trait-based predictions
    rank = list(reversed(sorted([round(x, 5) for x in result[-1]]))).index(round(result[3], 5)) + 1
    #print(">>>", result[3], [x for x in reversed(sorted([round(x, 5) for x in result[-1]]))], rank)
    #input()
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

print("Rank Distribution", r1, r2, r3, r4, r5, r6)


from matplotlib import pyplot as plt
#plt.hist(ranks_random, bins=range(0, 121 + 1, 1), alpha=0.7, color="orange", label="Random Assignment")
plt.hist(ranks_tb, bins=range(-1, 8, 1), alpha=0.7, color="blue", label="Trait-Based Assignment")
plt.plot([-1, 10], [len(ranks_tb) / 6, len(ranks_tb) / 6], color="goldenrod")
plt.title("Histogram of rankings of trait-based and random assignment w.r.t. all (6) possible assignments, using only Stage 1 and Stage 2 [N=" + str(len(ranks_tb)) + "]")
plt.ylabel("# of trait-based assignments")
plt.xlabel("Rank of the assignment compared to all 6 possible assignments")
plt.legend()
plt.show()
