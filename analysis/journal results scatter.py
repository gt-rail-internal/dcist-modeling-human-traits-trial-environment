# analyzes the data to determine the results

from statistics import median
import allocation.assignment_util

import sys, os.path, pickle
# check if the file exists
if not os.path.isfile("f3c3fake_results.pkl"):
    print("No results file")
    sys.exit()

# load users
user_scores, slopes = allocation.assignment_util.generate_fake_user_scores(N=30, r=.95, trait_noise=0.4, task_noise=0)
print("2", list(user_scores.keys())[0])
# process scores into lists for each trait and task
s1 = []
s2 = []
s3 = []
ot = []
ni = []
sa = []
theory_ot = []

for p in user_scores:
    s1.append(user_scores[p]["s1"])
    s2.append(user_scores[p]["s2"])
    s3.append(user_scores[p]["s3"])
    ot.append(user_scores[p]["ot"])
    ni.append(user_scores[p]["ni"])
    sa.append(user_scores[p]["sa"])
    theory_ot.append(user_scores[p]["ot"] * slopes["s1"]["ot"] + 0.5)

# scatterplot each relationship
from matplotlib import pyplot as plt
traits = ["Object Tracking", "Network Inference", "Situational Awareness"]
tasks = ["Stage 1", "Stage 2", "Stage 3"]

fig, axes = plt.subplots(nrows=3, ncols=3)

axes[0][0].scatter(ot, s1)
axes[0][1].scatter(ni, s1)
axes[0][2].scatter(sa, s1)
axes[1][0].scatter(ot, s2)
axes[1][1].scatter(ni, s2)
axes[1][2].scatter(sa, s2)
axes[2][0].scatter(ot, s3)
axes[2][1].scatter(ni, s3)
axes[2][2].scatter(sa, s3)

for row in range(len(axes)):
    for col in range(len(axes[row])):
        axes[row][col].set_xlabel(traits[col])
        axes[row][col].set_ylabel(tasks[row])
        axes[row][col].set_xlim([0,1])
        axes[row][col].set_ylim([0,1])

ax = plt.gca()  # get the plot axis

plt.show()
