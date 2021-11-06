# analyzes the data to determine the results

from statistics import median, stdev
import allocation.assignment_util

# calculate the pairwise ranking, plot distribution of performance difference between actual scores and trait based
# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]
prediction_tasks = tasks

# process the data into user scores matrix
specific_users = []
user_scores = allocation.assignment_util.process_logs("logs", specific_users=specific_users)  # all user scores (even those who failed to have complete data)
complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data

s1_pred = []
s1_adj = []
s1_actual = []

s2_pred = []
s2_adj = []
s2_actual = []

s3_pred = []
s3_adj = []
s3_actual = []

# determine the predicted and actual scores for each user
for user_id in complete_user_scores:
    # extract the test/train user IDs from the team indexes
    training_ids = [x for x in user_scores if x != user_id]  # use the remaining IDs for testing (one-hot)

    # pull the test/train data using the user IDs
    test_user_scores = {user_id : user_scores[user_id]}  # get testing subset of user scores
    training_user_scores = {p : user_scores[p] for p in training_ids}  # get training subset of user scores

    # generate impact matrix for the nine trait-task pairings, using the training user scores
    impact_matrix, yint_matrix = allocation.assignment_util.generate_impact_matrix(training_user_scores, traits=traits, tasks=tasks)

    # use the impact matrix to get the predicted score for the test user
    score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix={}, traits=traits, tasks=prediction_tasks)  # predicted user scores for each task
    score_adjusted_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix=yint_matrix, traits=traits, tasks=tasks)  # predicted user scores for each task INCLUDING Y INTERCEPT (not used by algorithm)
    score_actual_matrix = {p : {task : user_scores[p][task] for task in tasks} for p in test_user_scores}  # actual user scores for each task

    s1_pred.append(score_prediction_matrix[user_id]["s1"])
    s1_adj.append(score_adjusted_prediction_matrix[user_id]["s1"])
    s1_actual.append(user_scores[user_id]["s1"])

    s2_pred.append(score_prediction_matrix[user_id]["s2"])
    s2_adj.append(score_adjusted_prediction_matrix[user_id]["s2"])
    s2_actual.append(user_scores[user_id]["s2"])

    s3_pred.append(score_prediction_matrix[user_id]["s3"])
    s3_adj.append(score_adjusted_prediction_matrix[user_id]["s3"])
    s3_actual.append(user_scores[user_id]["s3"])

total_pred = [s1_pred[i] + s2_pred[i] + s3_pred[i] for i in range(len(s1_pred))]
total_adj = [s1_adj[i] + s2_adj[i] + s3_adj[i] for i in range(len(s1_adj))]
total_actual = [s1_actual[i] + s2_actual[i] + s3_actual[i] for i in range(len(s1_actual))]

total_s1s2_pred = [s1_pred[i] + s2_pred[i] for i in range(len(s1_pred))]

print(s1_pred)

# format data into scatter plots
from matplotlib import pyplot as plt
import itertools

# plot the trait-based predicted values vs the actual scores
f, axs = plt.subplots(nrows=2, ncols=2)
#axs[0, 0].scatter(x=s1_pred,y=s1_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores
#axs[1, 0].scatter(x=s2_pred,y=s2_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores
axs[0, 0].scatter(x=s3_pred,y=s3_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores
axs[1, 0].scatter(x=total_s1s2_pred,y=total_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores
#axs[0, 1].scatter(x=s1_adj,y=s1_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores
#axs[1, 1].scatter(x=s2_adj,y=s2_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores
axs[0, 1].scatter(x=s3_adj,y=s3_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores
axs[1, 1].scatter(x=total_adj,y=total_actual, color="blue", label="Stage 1 Actual vs. Predicted")  # histogram for best scores

for i in range(len(axs)):
    for j in range(len(axs[i])):
        axs[i, j].set_title(("Stage " + str(i+1) if i != 1 else "Total Score") + " Actual vs. " + ("Predicted" if j == 0 else "Adj. Predicted"))
        axs[i, j].set_xlim(([0, 1.1] if i != 1 else [0, 3.1]))
        axs[i, j].set_ylim(([0, 1.1] if i != 1 else [0, 3.1]))

plt.show()