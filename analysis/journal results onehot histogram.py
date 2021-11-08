# analyzes the data to determine the results

from statistics import median

from matplotlib.pyplot import plot
import allocation.assignment_util

# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]
prediction_tasks = tasks

# process the data into user scores matrix
specific_users = []
#user_scores = allocation.assignment_util.process_logs("logs", specific_users=specific_users)  # all user scores (even those who failed to have complete data)
complete_user_scores, slopes = allocation.assignment_util.generate_fake_user_scores(N=30, trait_noise=0.2, task_noise=1.0)
#complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data

# generate the one hot of test and trait data
import random
import allocation.onehot_allocation

num_train = len(complete_user_scores) - len(tasks)  # the number of items to train on
num_test = len(tasks)  # the number of items to test on

complete_user_scores_ids = list(complete_user_scores.keys())  # pull the IDs of the users who completed all stages
team_indexes = allocation.assignment_util.pullSubsets(len(complete_user_scores_ids), len(tasks))  # pull each possible team, in the form of indexes instead of IDs

# train and evaluate on each fold
score_data = []  # holds the data for each team combination
for team in team_indexes:
    # extract the test/train user IDs from the team indexes
    test_ids = [complete_user_scores_ids[i] for i in team]  # convert the team indexes to user IDs
    score_data.append(onehot_allocation.onehot_allocation(complete_user_scores, test_ids, traits, tasks, prediction_tasks))  # record the score data

# save the iteration scores
import pickle

with open("f3c3fake_results.pkl", "wb") as f:
    pickle.dump(score_data, f)

# plot the histogram
import plotting.plot_histogram
fig = plotting.plot_histogram.plot_histogram(iteration_scores=score_data)

import matplotlib.pyplot as plt
plt.show()