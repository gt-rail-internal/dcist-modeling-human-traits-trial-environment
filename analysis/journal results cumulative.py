# analyzes the data to determine the results

from statistics import median
import allocation.assignment_util
import plotting.plot_scatter
import plotting.plot_histogram
import plotting.plot_lines
import plotting.plot_ranking
import plotting.plot_whiskers
from matplotlib import pyplot as plt

import sys, os.path, pickle
# check if the file exists
if not os.path.isfile("f3c3fake_results.pkl"):
    print("No results file")
    sys.exit()


# load users
#user_scores = allocation.assignment_util.generate_fake_bivariate_user_scores(N=30, R=.0)
user_scores = allocation.assignment_util.process_logs()

# zero out the s3 stage
#user_scores = allocation.onehot_allocation.cut_task(user_scores, "s3")
#user_scores = allocation.assignment_util.oracle_task(user_scores, "s1", "ot")
#user_scores = allocation.assignment_util.oracle_task(user_scores, "s2", "ni")
#user_scores = allocation.assignment_util.oracle_task(user_scores, "s3", "sa")


complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores=user_scores)

correlation = "spearman"
impact_matrix, yint_matrix, weight_matrix = allocation.assignment_util.generate_impact_matrix(complete_user_scores, traits=["sa", "ni", "ot"], tasks=["s1", "s2", "s3"], correlation=correlation)

# plot the scatter plot
fig_scatter = plotting.plot_scatter.plot_scatter(user_scores, weight_matrix, correlation=correlation)  # correlation variable is only for text

# plot the line plot
plotting.plot_lines.plot_lines(user_scores)

# generate the scores for each 3 users (one hot)
print("generating user scores")
score_data = allocation.assignment_util.process_users(user_scores=user_scores, complete_user_scores=user_scores, traits=["ot", "ni", "sa"], tasks=["s1", "s2", "s3"])

# plot the onehot histogram
fig_histogram = plotting.plot_histogram.plot_histogram(score_data, R=-1)

# plot the 3c3 ranking
fig_ranking3c3 = plotting.plot_ranking.plot_ranking_3c3(score_data=score_data, R=-1)

# plot the 3c3 whiskers
fig_whiskers3c3 = plotting.plot_whiskers.plot_whiskers_3c3(score_data=score_data, R=-1)

plt.show()
