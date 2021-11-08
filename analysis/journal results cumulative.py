# analyzes the data to determine the results

from statistics import median
import allocation.assignment_util
import plotting.plot_scatter
import plotting.plot_histogram
from matplotlib import pyplot as plt

import sys, os.path, pickle
# check if the file exists
if not os.path.isfile("f3c3fake_results.pkl"):
    print("No results file")
    sys.exit()

# set parameters
task_noise = 0.1

# load users
user_scores, slopes = allocation.assignment_util.generate_fake_user_scores(N=30, trait_noise=0.2, task_noise=task_noise)

# plot the scatter plot
fig_scatter = plotting.plot_scatter.plot_scatter(user_scores)

# generate the scores for each 3 users (one hot)
score_data = allocation.assignment_util.process_users(user_scores=user_scores, complete_user_scores=user_scores, traits=["ot", "ni", "sa"], tasks=["s1", "s2", "s3"])

# plot the onehot histogram
fig_histogram = plotting.plot_histogram.plot_histogram(score_data, task_noise=task_noise)


plt.show()
