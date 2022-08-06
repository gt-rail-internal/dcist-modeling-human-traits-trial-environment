# analyzes the data to generate a line plot of predicted score vs actual score
import allocation.assignment_util

# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]

# process the data into user scores matrix
specific_users = ["1162", "3462", "4122", "5292"]
user_scores = allocation.assignment_util.process_logs("logs", specific_users=specific_users)  # all user scores (even those who failed to have complete data)
complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data

# generate impact matrix for the nine trait-task pairings, using the training user scores
impact_matrix = allocation.assignment_util.generate_impact_matrix(complete_user_scores, traits=traits, tasks=tasks)[0]

# use the impact matrix to get the predicted score for each of the test users
score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(complete_user_scores, impact_matrix=impact_matrix, traits=traits, tasks=tasks)  # predicted user scores for each task
score_actual_matrix = {p : {task : user_scores[p][task] for task in tasks} for p in complete_user_scores}  # actual user scores for each task

# plot the predicted scores vs the actual scores 
import matplotlib.pyplot as plt

# create three plots
fig, (ax1, ax2, ax3) = plt.subplots(1, 3)  # get the plot axis
axes = {"s1": ax1, "s2": ax2, "s3": ax3}

# initialize the plots
for stage in axes:
    axes[stage].set_title("[STAGE " + stage[-1] + "] Line Plot of Predicted Scores and Actual Scores")
    axes[stage].set_xlabel("Score Type")  # set x label
    axes[stage].set_xticks([1, 2])  # set locations of x ticks
    axes[stage].set_xticklabels(["Predicted", "Actual"])  # set labels of x ticks
    axes[stage].set_ylabel("Scaled Scores")  # set y label
    axes[stage].set_ylim([-.05, 1.05])  # set the y axes points

# format data into three lists of points (Stage 1, Stage 2, Stage 3)
for stage in axes:
    # get scaling for the score actual matrix
    actual_adjustment = min([score_actual_matrix[p][stage] for p in score_actual_matrix])
    actual_scale = max([score_actual_matrix[p][stage] - actual_adjustment for p in score_actual_matrix])

    # get scaling for the score prediction matrix
    pred_adjustment = min([score_prediction_matrix[p][stage] for p in score_prediction_matrix])
    pred_scale = max([score_prediction_matrix[p][stage] - pred_adjustment for p in score_prediction_matrix])

    for p in score_prediction_matrix:
        x = [1, 2]
        y = [(score_prediction_matrix[p][stage] - pred_adjustment) / pred_scale, (score_actual_matrix[p][stage] - actual_adjustment) / actual_scale]
        axes[stage].plot(x, y)

plt.show()
