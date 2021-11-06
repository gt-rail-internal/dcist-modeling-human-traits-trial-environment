# analyzes the data to determine the results

from statistics import median
import allocation.assignment_util

import sys, os.path, pickle
# check if the file exists
if not os.path.isfile("f3c3fake_results.pkl"):
    print("No results file")
    sys.exit()

# load users
user_scores, slopes = allocation.assignment_util.generate_fake_user_scores(N=30, trait_noise=0.4, task_noise=0)
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
    print(">>", user_scores[p]["ot"] * slopes["s1"]["ot"] + 0.5, user_scores[p]["s1"])

for i in range(len(s1)):
    print(ot[i], slopes["s1"]["ot"] * ot[i] + 0.5, s1[i])

# scatterplot each relationship
from matplotlib import pyplot as plt
ax = plt.gca()  # get the plot axis

plot_ot_s1 = ax.scatter(ot, s1)
plot_ottheory_s1 = ax.scatter(ot, theory_ot)

plt.show()
