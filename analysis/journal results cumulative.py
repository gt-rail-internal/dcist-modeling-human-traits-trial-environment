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
R = .95  # correlation of generated users

# load users
user_scores = allocation.assignment_util.generate_fake_bivariate_user_scores(N=30, R=R, slope_range=[0,.5])
#user_scores = allocation.assignment_util.process_logs()
#complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores=user_scores)

# plot the scatter plot
fig_scatter = plotting.plot_scatter.plot_scatter(user_scores)

# generate the scores for each 3 users (one hot)
score_data = allocation.assignment_util.process_users(user_scores=user_scores, complete_user_scores=user_scores, traits=["ot", "ni", "sa"], tasks=["s1", "s2", "s3"])

# plot the onehot histogram
fig_histogram = plotting.plot_histogram.plot_histogram(score_data, R=R)


plt.show()
