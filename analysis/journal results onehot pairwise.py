# analyzes the data to determine the results

from statistics import median, stdev
import allocation.assignment_util

# calculate the pairwise ranking, plot distribution of performance difference between actual scores and trait based

import sys, os.path, pickle
# check if the file exists
if not os.path.isfile("f3c3_results.pkl"):
    print("No results file")
    sys.exit()

# load the list
with open("f3c3_results.pkl", "rb") as f:
    iteration_scores = pickle.load(f)

# format data into histogram distributions
from matplotlib import pyplot as plt
import itertools
data = [[x[i] for x in iteration_scores] for i in range(len(iteration_scores[0])-1)]  # -1 to not consider the last (all possibilities)
all_assignments = list(itertools.chain.from_iterable([x[-1] for x in iteration_scores]))
print("num of teams selected", len(data[-1]))

# as an aside, determine how trait-based compares to all-assignments within a 3-user group
tb_wins = 0  # count of trait-based wins
all_wins = 0  # count of all-assignment wins
tie = 0  # count of ties
todos = [x[-1] for x in iteration_scores]  # all possible assignments for each set of 3 users (6)
differences = []
for d in range(len(data[0])):  # for each set of 3 users
    tb = data[3][d]  # pull their trait-based score
    for all in todos[d]:  # for each possible score for that set
        if tb > all:  # if trait-based wins
            tb_wins += 1
        if tb == all:  # if they tie (should be >= num of 3 user sets, 3276)
            tie += 1
        if tb < all:  # if all-assignment wins
            all_wins += 1
        #print("TB", tb, "ALL", all)

        if tb != all:
            differences.append(tb - all)
        
print("num", len(differences), "TB", tb_wins, "ALL", all_wins, "TIE", tie)

ax = plt.gca()  # get the plot axis
bins = 50
alpha = 0.5
range = [-3, 3]

# calculate the mean
mean = sum(differences) / len(differences)
std = stdev(differences)
print("std", std)
# plot
plot = ax.hist(differences, bins=bins, range=range, alpha=alpha, color="purple", label="Trait-Based Score - Possible Score")  # histogram for best scores
plt.axvline(mean, color='k', linestyle='dashed', linewidth=1, label="Mean difference = " + str(round(mean, 3)))
ax.legend()
ax.set_title("[Assigning 3 Users ONLY STAGE 1 and STAGE 2] Histogram of Within-Group Score Differences Between Trait-Based Assignment and All Possible Assignments, for " + str(len(data[0])) + " teams")
ax.set_xlabel("Difference in Score Between Trait-Based Assignment and All Possible Assignments")  # set x label
ax.set_xlim([-3, 3])  # set locations of x ticks
ax.set_ylabel("Number of Possible Assignments\n(6 Assignments per Team)")  # set y label
plt.show()

