# analyzes the data to determine the results

from statistics import median
import allocation.assignment_util

PLOTTING = True  # flag for if we are plotting the results OR calculating them

# check if we have a results, if so display them
if PLOTTING:
    import sys, os.path, pickle
    # check if the file exists
    if not os.path.isfile("f6c3_results.pkl"):
        print("No results file")
        sys.exit()

    # load the list
    with open("f6c3_results.pkl", "rb") as f:
        iteration_scores = pickle.load(f)
    
    # format data into histogram distributions
    from matplotlib import pyplot as plt
    data = [[x[i] for x in iteration_scores] for i in range(len(iteration_scores[0]))]

    ax = plt.gca()  # get the plot axis
    bins = 50
    alpha = 0.5
    range = [0, 3]
    plot = ax.hist(data[0], bins=bins, range=range, alpha=alpha, color="green", label="Known Best")  # histogram for best scores
    plot = ax.hist(data[1], bins=bins, range=range, alpha=alpha, color="blue", label="Trait-Based Predicted")  # histogram for predicted actual
    plot = ax.hist(data[3], bins=bins, range=range, alpha=alpha, color="orange", label="Expected Value")  # histogram for expected scores
    plot = ax.hist(data[2], bins=bins, range=range, alpha=alpha, color="red", label="Known Worst")  # histogram for worst scores
    
    ax.legend()
    ax.set_title("[From 6 Choose 3] Histogram of Scores via Assignment Methods")
    ax.set_xlabel("Team Assignment Score")  # set x label
    ax.set_xlim([0, 3])  # set locations of x ticks
    ax.set_ylabel("Number of Teams")  # set y label
    plt.show()


# define the trait and tasks
traits=["ot", "sa", "ni"]
tasks=["s1", "s2", "s3"]

import os, random

# for N iterations, choose 6 users, get best predicted team of 3, compare to actual team of 3
N = 5000

used_samplings = []

iteration_scores = []
for n in range(N):
    print("#", n)
    # randomly choose 6 users

    all_users = [x.split(".")[0] for x in os.listdir("./logs") if "-" not in x and "email" not in x]  # get user names

    # sample a user set
    while True:
        specific_users = random.sample(all_users, 6)  # select 6
        if specific_users not in used_samplings:
            used_samplings.append(specific_users)
            break
    

    # process the data into user scores matrix
    user_scores = allocation.assignment_util.process_logs("logs", specific_users=specific_users)  # all user scores (even those who failed to have complete data)
    complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data

    complete_user_scores_ids = list(complete_user_scores.keys())  # pull the IDs of the users who completed all stages
    team_indexes = allocation.assignment_util.pullSubsets(len(specific_users), len(tasks))  # pull each possible team, in the form of indexes instead of IDs

    # train and evaluate on each fold
    score_data = []  # holds the data for each team combination
    for team in team_indexes:
        # extract the test/train user IDs from the team indexes
        test_ids = [specific_users[i] for i in team]  # convert the team indexes to user IDs
        training_ids = [x for x in user_scores if x not in test_ids]  # use the remaining IDs for testing (one-hot)

        # pull the test/train data using the user IDs
        test_user_scores = {p : user_scores[p] for p in test_ids}  # get testing subset of user scores
        training_user_scores = {p : user_scores[p] for p in training_ids}  # get training subset of user scores

        # generate impact matrix for the nine trait-task pairings, using the training user scores
        impact_matrix, yint_matrix = allocation.assignment_util.generate_impact_matrix(training_user_scores, traits=traits, tasks=tasks)

        # use the impact matrix to get the predicted score for each of the test users
        score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix={}, traits=traits, tasks=tasks)  # predicted user scores for each task
        score_adjusted_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix=yint_matrix, traits=traits, tasks=tasks)  # predicted user scores for each task INCLUDING Y INTERCEPT (not used by algorithm)
        score_actual_matrix = {p : {task : user_scores[p][task] for task in tasks} for p in test_user_scores}  # actual user scores for each task
        #print("Y", yint_matrix)
        #print(">", score_prediction_matrix)
        #print("#", score_adjusted_prediction_matrix)

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

        # get the expected value for the team
        #print("TEAM", test_ids)
        expected_value = 0
        expected_value += user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[2]][tasks[2]]
        #print(">012", user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[2]][tasks[2]])
        expected_value += user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[1]][tasks[2]]
        #print(">021", user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[1]][tasks[2]])
        expected_value += user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[0]][tasks[2]]
        #print(">120", user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[0]][tasks[2]])
        expected_value += user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[2]][tasks[2]]
        #print(">102", user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[2]][tasks[2]])
        expected_value += user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[1]][tasks[2]]
        #print(">201", user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[1]][tasks[2]])
        expected_value += user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[0]][tasks[2]]
        #print(">210", user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[0]][tasks[2]])
        expected_value /= 6

        score_data.append([best_value, pred_value, adjusted_pred_value, actual_value, worst_value, expected_value])  # record the score data
        
    # isolate the best teams
    import math
    best_team = -1
    best_score = -1
    pred_team = -1
    pred_score = -1
    adjusted_pred_score = -1
    actual_score = -1
    worst_team = -1
    worst_score = float("inf")
    expected_value = 0
    for i in range(len(score_data)):
        if score_data[i][0] > best_score:
            best_score = score_data[i][0]
            best_team = i
        if score_data[i][1] > pred_score:
            pred_score = score_data[i][1]
            adjusted_pred_score = score_data[i][2]
            actual_score = score_data[i][3]
            pred_team = i
        if score_data[i][4] < worst_score:
            worst_score = score_data[i][4]
            worst_team = i
        expected_value += score_data[i][5]
    expected_value /= len(score_data)
    

    iteration_scores.append([best_score, actual_score, worst_score, expected_value])
    #print("TEAM OUTCOME")
    #print("Chosen 6", specific_users)
    #print("Best team", [specific_users[x] for x in team_indexes[best_team]], "(best)", round(score_data[best_team][0], 2), "(pred)", round(score_data[best_team][1], 2), "(adj pred)", round(score_data[best_team][2], 2), "(actual)", round(score_data[best_team][3], 2), "(worst)", round(score_data[best_team][4], 2), "(exp)", round(score_data[best_team][5], 2))
    #print("Pred team", [specific_users[x] for x in team_indexes[pred_team]], "(best)", round(score_data[pred_team][0], 2), "(pred)", round(score_data[pred_team][1], 2), "(adj pred)", round(score_data[pred_team][2], 2), "(actual)", round(score_data[pred_team][3], 2), "(worst)", round(score_data[pred_team][4], 2), "(exp)", round(score_data[pred_team][5], 2))
    #print("Worst team", [specific_users[x] for x in team_indexes[worst_team]], "(best))", round(score_data[worst_team][0], 2), "(pred)", round(score_data[worst_team][1], 2), "(adj pred)", round(score_data[worst_team][2], 2), "(actual)", round(score_data[worst_team][3], 2), "(worst)", round(score_data[worst_team][4], 2), "(exp)", round(score_data[worst_team][5], 2))
    #print("Expected value", round(expected_value, 2))
    

    # generate averages
    print("AVG best", sum([x[0] for x in iteration_scores]) / len(iteration_scores), "AVG pred", sum([x[1] for x in iteration_scores]) / len(iteration_scores), "AVG worst", sum([x[2] for x in iteration_scores]) / len(iteration_scores), "AVG exp", sum([x[3] for x in iteration_scores]) / len(iteration_scores))

# save the iteration scores
import pickle

with open("f6c3_results.pkl", "wb") as f:
    pickle.dump(iteration_scores, f)

print("done")