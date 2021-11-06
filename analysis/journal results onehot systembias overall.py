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

print(data[1][100:120])

# plot the trait-based predicted values vs the actual scores
ax = plt.scatter(x=data[1],y=data[2], color="blue", label="Trait-Based Score - Possible Score")  # histogram for best scores
plt.legend()
plt.title("[Assigning 3 Users ONLY STAGE 1 and STAGE 2] Histogram of Within-Group Score Differences Between Trait-Based Assignment and All Possible Assignments, for " + str(len(data[0])) + " teams")
plt.xlabel("Difference in Score Between Trait-Based Assignment and All Possible Assignments")  # set x label
plt.ylabel("Number of Possible Assignments\n(6 Assignments per Team)")  # set y label
plt.show()
