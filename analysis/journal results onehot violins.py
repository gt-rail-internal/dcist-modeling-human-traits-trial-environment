# analyzes the data to determine the results

from statistics import median
import allocation.assignment_util

# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]

# process the data into user scores matrix
specific_users = []
user_scores = allocation.assignment_util.process_logs("logs", specific_users=specific_users)  # all user scores (even those who failed to have complete data)
complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data

# generate the one hot of test and trait data
import random

num_train = len(complete_user_scores) - len(tasks)  # the number of items to train on
num_test = len(tasks)  # the number of items to test on

complete_user_scores_ids = list(complete_user_scores.keys())  # pull the IDs of the users who completed all stages
team_indexes = allocation.assignment_util.pullSubsets(len(complete_user_scores_ids), len(tasks))  # pull each possible team, in the form of indexes instead of IDs

# train and evaluate on each fold
score_data = []  # holds the data for each team combination
for team in team_indexes:
    # extract the test/train user IDs from the team indexes
    test_ids = [complete_user_scores_ids[i] for i in team]  # convert the team indexes to user IDs
    training_ids = [x for x in user_scores if x not in test_ids]  # use the remaining IDs for testing (one-hot)

    # pull the test/train data using the user IDs
    test_user_scores = {p : user_scores[p] for p in test_ids}  # get testing subset of user scores
    training_user_scores = {p : user_scores[p] for p in training_ids}  # get training subset of user scores

    # generate impact matrix for the nine trait-task pairings, using the training user scores
    impact_matrix = allocation.assignment_util.generate_impact_matrix(training_user_scores, traits=traits, tasks=tasks)[0]

    # use the impact matrix to get the predicted score for each of the test users
    score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, traits=traits, tasks=tasks)  # predicted user scores for each task
    score_actual_matrix = {p : {task : user_scores[p][task] for task in tasks} for p in test_user_scores}  # actual user scores for each task

    # for each team member, get the predicted best score and the actual best score

    # get the predicted and actual scores for humans in this team
    pred = [[p[task] for task in tasks] for p in [score_prediction_matrix[x] for x in test_ids]]
    actual = [[p[task] for task in tasks] for p in [score_actual_matrix[x] for x in test_ids]]
    
    # determine their optimal assignments
    best_a = allocation.assignment_util.hungarian(actual, maximize=True)
    pred_a = allocation.assignment_util.hungarian(pred, maximize=True)
    worst_a = allocation.assignment_util.hungarian(actual, maximize=False)
    rand_a = random.sample(range(len(tasks)), len(team))
    rand_a_2 = random.sample(range(len(tasks)), len(team))

    # record the actual scores from each assignment for analysis
    best_value = 0
    pred_value = 0
    worst_value = 0
    rand_value = 0
    rand_value_2 = 0

    # for each index of the subset
    for i in range(len(team)):
        idx = test_ids[i]
        best_value += user_scores[idx][tasks[best_a[i]]]  # add the actual score to the value of that team assignment
        pred_value += user_scores[idx][tasks[pred_a[i]]]  # add the predicted score to the value of that team assignment
        worst_value += user_scores[idx][tasks[worst_a[i]]]  # add the worst score to the value of that team assignment
        rand_value += user_scores[idx][tasks[rand_a[i]]]  # add the random score to the value of that team assignment
        rand_value_2 += user_scores[idx][tasks[rand_a_2[i]]]

    denom = best_value - worst_value

    best_numer = best_value - worst_value
    pred_numer = pred_value - worst_value
    worst_numer = worst_value - worst_value
    rand_numer = rand_value - worst_value
    rand_numer_2 = rand_value_2 - worst_value
    score_data.append([best_numer / denom, pred_numer / denom, worst_numer / denom, rand_numer / denom, rand_numer_2 / denom])  # record the score data
    

# plot the results 
import matplotlib.pyplot as plt

# format data into four lists of points
data = [[x[i] for x in score_data] for i in range(len(score_data[0]))]

# combine the last two datasets (two random instances)
data[3] = data[3] + data[4]
del data[4]

# also double the predicted data
data[1] = data[1] + data[1]

# randomness sanity check
correct_rand = sum([1 for x in data[3] if x == 1])
total_rand = len(data[3])
print("worst ratio", round(sum([1 for x in data[3] if x == 0]) / len(data[3]), 3), "correct rand", correct_rand, "total rand", total_rand, "ratio", round(correct_rand / total_rand, 4), "mean", sum(data[3]) / len(data[3]), "median", median(data[3]))

# predicted results
correct_pred = sum([1 for x in data[1] if x == 1])
total_pred = len(data[1])
print("worst ratio", round(sum([1 for x in data[1] if x == 0]) / len(data[1]), 3), "correct pred", correct_pred, "total pred", total_pred, "ratio", round(correct_pred / total_pred, 4), "mean", sum(data[1]) / len(data[1]), "median", median(data[1]))

# FOR DCIST QUARTERLY
import seaborn as sns
ax = sns.violinplot(data=[data[1], data[3]], scale="area")
ax.set_title("Team performance scaled from their Known Worst to Known Optimal performances, using Trait-Based and Random Assignment (N=" + str(len(data[0])) + ")")
ax.set_xlabel("Team Assignment Type")  # set x label
ax.set_xticks([0, 1])  # set locations of x ticks
ax.set_xticklabels(["Trait-Based", "Random"])  # set labels of x ticks
ax.set_ylabel("Team Performance with respect to each Team's Known Worst and Known Optimal assignments\n(higher is better)")  # set y label
ax.set_ylim([0, 1])
plt.show()

"""
ax = plt.gca()  # get the plot axis
plot = ax.violinplot(data)  # create the violin plot
ax.set_title("[Leave One Out Sampling] Team Task Assignment Scores compared to Known Best and Known Worst vs. Assignment Methods, N=" + str(len(data[0])))
ax.set_xlabel("Team Assignment Type")  # set x label
ax.set_xticks([1, 2, 3, 4])  # set locations of x ticks
ax.set_xticklabels(["Known Best", "Trait-Based", "Known Worst", "Random"])  # set labels of x ticks
ax.set_ylabel("Retroactive Team Scores\n[team score scaled between Known Best and Known Worst]")  # set y label
ax.set_ylim([0, 1.05])
for i in range(len(plot["bodies"])):
    plot["bodies"][i].set_facecolor(["green", "blue", "black", "purple"][i])
    plot["bodies"][i].set_edgecolor("black")
    plot["cbars"].set_edgecolor("grey")
    plot["cmins"].set_edgecolor("grey")
    plot["cmaxes"].set_edgecolor("grey")
    plot["bodies"][i].set_alpha(.3)
plt.show()
"""