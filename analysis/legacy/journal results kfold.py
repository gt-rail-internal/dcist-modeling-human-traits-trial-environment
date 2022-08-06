# analyzes the data to determine the results

import allocation.assignment_util

# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]

# process the data into user scores matrix
user_scores = allocation.assignment_util.process_logs("logs")  # all user scores (even those who failed to have complete data)
complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data

# generate the k-fold of test and trait data
import random

K = 4
train_ratio = 1 - 1 / K  # proportion of the dataset to train on (generate trendlines with)
num_train = int(len(complete_user_scores) * train_ratio)  # the number of items to train on
num_test = int(len(complete_user_scores) * (1 - train_ratio))

randomized_complete_user_ids = list(complete_user_scores.keys())
random.shuffle(randomized_complete_user_ids)  # randomized IDs of users who completed all stages

# train and evaluate on each fold
score_data = []  # holds the data for each team combination
for kfold in range(int(1 / (1 - train_ratio))):
    test_ids = randomized_complete_user_ids[kfold * num_test : (kfold + 1) * num_test]  # sample user IDs for training
    training_ids = [x for x in user_scores if x not in test_ids]  # use the remaining IDs for testing
    training_user_scores = {p : user_scores[p] for p in training_ids}  # get training subset of user scores
    test_user_scores = {p : user_scores[p] for p in test_ids}  # get testing subset of user scores

    # generate impact matrix for the nine trait-task pairings, using the training user scores
    impact_matrix = allocation.assignment_util.generate_impact_matrix(training_user_scores, traits=traits, tasks=tasks)[0]

    # use the impact matrix to get the predicted score for each of the test users
    score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, traits=traits, tasks=tasks)  # predicted user scores for each task
    score_actual_matrix = {p : {task : user_scores[p][task] for task in tasks} for p in test_user_scores}  # actual user scores for each task

    # pull subsets of humans from the test pool
    subset_indexes = allocation.assignment_util.pullSubsets(len(test_ids), len(tasks))
    subset_ids = [[test_ids[p] for p in subset] for subset in subset_indexes]

    # for each subset, get the predicted best score and the actual best score
    for s in subset_ids:
        # get the predicted and actual scores for humans in this subset
        pred = [[p[task] for task in tasks] for p in [score_prediction_matrix[x] for x in s]]
        actual = [[p[task] for task in tasks] for p in [score_actual_matrix[x] for x in s]]
        
        # determine their optimal assignments
        best_a = allocation.assignment_util.hungarian(actual, maximize=True)
        pred_a = allocation.assignment_util.hungarian(pred, maximize=True)
        worst_a = allocation.assignment_util.hungarian(actual, maximize=False)
        rand_a = random.sample(range(len(tasks)), len(s))

        # record the actual scores from each assignment for analysis
        best_value = 0
        pred_value = 0
        worst_value = 0
        rand_value = 0
        # for each index (user) of the subset
        for i in range(len(s)):
            user_id = s[i]  # the user ID of this person

            best_value += user_scores[user_id][tasks[best_a[i]]]  # add the actual score to the value of that team assignment
            pred_value += user_scores[user_id][tasks[pred_a[i]]]  # add the predicted score to the value of that team assignment
            worst_value += user_scores[user_id][tasks[worst_a[i]]]  # add the worst score to the value of that team assignment
            rand_value += user_scores[user_id][tasks[rand_a[i]]]  # add the random score to the value of that team assignment

            #print("BEST", best_value, pred_value, worst_value, rand_value)

        score_data.append([best_value / best_value, pred_value / best_value, worst_value / best_value, rand_value / best_value])  # record the score data
        

# plot the results 
import matplotlib.pyplot as plt

# format data into four lists of points
data = [[x[i] for x in score_data] for i in range(len(score_data[0]))]

ax = plt.gca()  # get the plot axis
plot = ax.violinplot(data)  # create the violin plot
ax.set_title("[K-Fold Sampling] Task Assignment Scores via Assignment Methods, K-Fold K=" + str(K) + " N=" + str(len(data[0])))
ax.set_xlabel("Team Assignment Type")  # set x label
ax.set_xticks([1, 2, 3, 4])  # set locations of x ticks
ax.set_xticklabels(["Known Best", "Trait-Based", "Known Worst", "Random"])  # set labels of x ticks
ax.set_ylabel("Retroactive Team Scores\n[proportion of teams' Known Best score]")  # set y label
for i in range(len(plot["bodies"])):
    plot["bodies"][i].set_facecolor(["green", "blue", "black", "purple"][i])
    plot["bodies"][i].set_edgecolor("black")
    plot["cbars"].set_edgecolor("grey")
    plot["cmins"].set_edgecolor("grey")
    plot["cmaxes"].set_edgecolor("grey")
    plot["bodies"][i].set_alpha(.3)
plt.show()