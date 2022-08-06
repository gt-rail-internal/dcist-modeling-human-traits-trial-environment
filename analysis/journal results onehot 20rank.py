# analyzes the data to determine the results

from statistics import median, stdev
import allocation.assignment_util

# calculate the pairwise ranking, plot distribution of performance difference between actual scores and trait based

import sys, os.path, pickle

# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]
prediction_tasks = ["s1", "s2"]

import os, random

# for N iterations, choose 6 users, get best predicted team of 3, compare to actual team of 3
N = 1000

used_samplings = []  # list of 6-user samples we have already looked at
iteration_scores = []

# pull the user scores from the logs (or generate them randomly)
user_scores = allocation.assignment_util.process_logs("logs")  # all user scores (even those who failed to have complete data)
#user_scores = allocation.assignment_util.generate_fake_user_scores(N=30, trait_noise=0, task_noise=0)

# pull out the complete users (applicable for a 6-user subset)
complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data
complete_user_scores_ids = list(complete_user_scores.keys())  # pull the IDs of the users who completed all stages

iteration_scores = []  # holds the data for each team combination

for n in range(N):
    print("#", n)
    # randomly choose 6 users
    while True:
        specific_users = random.sample(complete_user_scores_ids, 6)  # select 6
        if specific_users not in used_samplings:
            used_samplings.append(specific_users)
            break
    
    best_of6 = []
    trait_of6 = 0
    actual_of6 = 0

    team_indexes = allocation.assignment_util.pullSubsets(len(specific_users), len(tasks))  # pull each possible team, in the form of indexes instead of IDs

    # train and evaluate on each three-user fold
    all_assignments = []
    for team in team_indexes:
        # extract the test/train user IDs from the team indexes
        test_ids = [specific_users[i] for i in team]  # convert the team indexes to user IDs
        training_ids = [x for x in user_scores if x not in test_ids]  # use the remaining IDs for testing (one-hot)

        # pull the test/train data using the user IDs
        test_user_scores = {p : user_scores[p] for p in test_ids}  # get testing subset of user scores
        training_user_scores = {p : user_scores[p] for p in training_ids}  # get training subset of user scores

        # generate impact matrix for the nine trait-task pairings, using the training user scores
        impact_matrix, yint_matrix, weight_matrix = allocation.assignment_util.generate_impact_matrix(training_user_scores, traits=traits, tasks=tasks)

        # use the impact matrix to get the predicted score for each of the test users
        score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix={}, traits=traits, tasks=prediction_tasks)  # predicted user scores for each task
        score_adjusted_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix=yint_matrix, traits=traits, tasks=tasks, weight_matrix=weight_matrix)  # predicted user scores for each task INCLUDING Y INTERCEPT (not used by algorithm)
        score_actual_matrix = {p : {task : user_scores[p][task] for task in tasks} for p in test_user_scores}  # actual user scores for each task

        # for each team member, get the predicted best score and the actual best score

        # get the predicted and actual scores for humans in this team
        pred = [[p[task] for task in tasks] for p in [score_prediction_matrix[x] for x in test_ids]]
        adjusted_pred = [[p[task] for task in tasks] for p in [score_adjusted_prediction_matrix[x] for x in test_ids]]
        actual = [[p[task] for task in tasks] for p in [score_actual_matrix[x] for x in test_ids]]
        
        # determine their optimal assignments
        best_a = allocation.assignment_util.hungarian(actual, maximize=True)
        pred_a = allocation.assignment_util.hungarian(pred, maximize=True)
        worst_a = allocation.assignment_util.hungarian(actual, maximize=False)

        # record the actual scores from each assignment for analysis
        best_value = 0
        pred_value = 0
        adjusted_pred_value = 0
        actual_value = 0
        worst_value = 0

        # for each index of the subset
        for i in range(len(team)):
            idx = test_ids[i]
            best_value += user_scores[idx][tasks[best_a[i]]]  # add the actual score to the value of that team assignment
            pred_value += pred[i][pred_a[i]] # add the predicted score to the value of that team assignment
            adjusted_pred_value += adjusted_pred[i][pred_a[i]]  # add the adjusted predicted value (+y intercepts)
            actual_value += user_scores[idx][tasks[pred_a[i]]]  # add the predicted score to the value of that team assignment
            worst_value += user_scores[idx][tasks[worst_a[i]]]  # add the worst score to the value of that team assignment

        # append the best team
        best_of6.append(best_value)

        # update the predicted/actual best score
        if pred_value > trait_of6:
            trait_of6 = pred_value
            actual_of6 = actual_value
            
        # get the expected value for the team
        score = user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[2]][tasks[2]]
        all_assignments.append(score)
        score = user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[1]][tasks[2]]
        all_assignments.append(score)
        score = user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[0]][tasks[2]]
        all_assignments.append(score)
        score = user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[2]][tasks[2]]
        all_assignments.append(score)
        score = user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[1]][tasks[2]]
        all_assignments.append(score)
        score = user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[0]][tasks[2]]
        all_assignments.append(score)

    # skip if traitbased score is 0 (incomplete users)
    if trait_of6 == 0:
        continue

    iteration_scores.append([trait_of6, actual_of6, best_of6, all_assignments])  # record the score data


# for one example user set, from 6 choose 3, rank trait based to the score distribution
ranks_tb = []
ranks_random = []
worst = 0
# for each f6c3 team
for team in iteration_scores:
    # pull their trait-based score and all the possible assignments (120)
    traitbased_score = team[1]
    all_assignments = team[-1]
    # determine the rank of the user
    rank_tb = len([x for x in all_assignments if x > traitbased_score]) + 1
    rank_random = len([x for x in all_assignments if x > random.choice(all_assignments)]) + 1
    ranks_tb.append(rank_tb)
    ranks_random.append(rank_random)
    if traitbased_score == min(all_assignments):
        worst += 1
    if rank_tb >= 118:
        print("worst>", rank_tb, traitbased_score, min(all_assignments))
    if rank_tb == 0:
        print("best")
print("# worst", worst)

from matplotlib import pyplot as plt
plt.hist(ranks_random, bins=range(0, 121 + 1, 1), alpha=0.7, color="orange", label="Random Assignment")
plt.hist(ranks_tb, bins=range(0, 121 + 1, 1), alpha=0.7, color="blue", label="Trait-Based Assignment")
plt.title("Histogram of rankings of trait-based and random assignment w.r.t. all (120) possible assignments [N=" + str(len(ranks_tb)) + "]")
plt.ylabel("# of trait-based / random assignments")
plt.xlabel("Rank of the assignment")
plt.legend()
plt.show()
