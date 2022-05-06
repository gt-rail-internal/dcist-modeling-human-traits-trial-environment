import allocation.assignment_util
import itertools


# zeros out a task
def cut_task(user_scores, task):
    new_scores = {}
    for p in user_scores:
        new_user = user_scores[p]
        new_user[task] = 0
        new_scores[p] = new_user
    return new_scores

# runs onehot allocation on a set of test users
def onehot_allocation(user_scores, test_ids, traits, tasks, prediction_tasks=[], include_yint=True, team_size=3):
    # if prediction tasks weren't provided, use all tasks
    if prediction_tasks == []:
        prediction_tasks = tasks

    # extract the test/train user IDs from the team indexes
    training_ids = [x for x in user_scores if x not in test_ids]  # use the remaining IDs for testing (one-hot)

    # pull the test/train data using the user IDs
    test_user_scores = {p : user_scores[p] for p in test_ids}  # get testing subset of user scores
    training_user_scores = {p : user_scores[p] for p in training_ids}  # get training subset of user scores

    # generate impact matrix for the nine trait-task pairings, using the training user scores, as well as the y intercept matrix and the weight matrix (correlation coefficients)
    impact_matrix, yint_matrix, weight_matrix = allocation.assignment_util.generate_impact_matrix(training_user_scores, traits=traits, tasks=tasks)
    #print(yint_matrix)
    # use the impact matrix to get the predicted score for each of the test users
    score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix={}, weight_matrix=weight_matrix, traits=traits, tasks=prediction_tasks)  # predicted user scores for each task
    score_adjusted_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix=yint_matrix, weight_matrix=weight_matrix, traits=traits, tasks=tasks)  # predicted user scores for each task INCLUDING Y INTERCEPT (not used by algorithm)

    score_actual_matrix = {p : {task : user_scores[p][task] for task in tasks} for p in test_user_scores}  # actual user scores for each task
    
    # for each team member, get the predicted best score and the actual best score


    # determine their optimal assignments
    # if the team_ids size does not match the team_size, we are sampling (e.g., from 6 choose 3), so consider all possible teams
    teams = [test_ids]  # by default, teams is test_ids
    if team_size < len(test_ids):  # if team size is less than test_ids, we are checking out subsets
        #print("generating subteams from test_ids")
        teams = [list(x) for x in itertools.combinations(test_ids, len(traits))]

    # record the actual scores from each assignment for analysis
    all_best_value = 0
    all_pred_value = 0
    all_adjusted_pred_value = 0
    all_actual_value = 0
    all_worst_value = 0
    all_expected_values = []
    all_assignments = []
    
    for team in teams:
        #print("allocation team", team)
        # get the predicted and actual scores for humans in this team
        pred = [[p[task] for task in tasks] for p in [score_prediction_matrix[x] for x in team]]
        adjusted_pred = [[p[task] for task in tasks] for p in [score_adjusted_prediction_matrix[x] for x in team]]
        actual = [[p[task] for task in tasks] for p in [score_actual_matrix[x] for x in team]]

        best_a = allocation.assignment_util.hungarian(actual, maximize=True)
        pred_a = allocation.assignment_util.hungarian(pred, maximize=True)
        adjusted_pred_a = allocation.assignment_util.hungarian(adjusted_pred, maximize=True)
        worst_a = allocation.assignment_util.hungarian(actual, maximize=False)

        best_value = 0
        pred_value = 0
        adjusted_pred_value = 0
        actual_value = 0
        worst_value = 0

        # for each index of the subset, add the score
        for i in range(len(team)):
            user_id = team[i]
            best_value += user_scores[user_id][tasks[best_a[i]]]  # add the actual score to the value of that team assignment
            pred_value += pred[i][pred_a[i]] # add the predicted score to the value of that team assignment
            adjusted_pred_value += adjusted_pred[i][pred_a[i]]  # add the adjusted predicted value (+y intercepts)
            actual_value += user_scores[user_id][tasks[adjusted_pred_a[i] if include_yint else pred_a[i]]]  # add the predicted score to the value of that team assignment
            worst_value += user_scores[user_id][tasks[worst_a[i]]]  # add the worst score to the value of that team assignment
            
        # get the expected value for the team
        expected_value = 0
        assignments = [list(x) for x in itertools.permutations(team, len(tasks))]

        # for each assignment, calculate score
        for assignment in assignments:
            score = sum([user_scores[assignment[i]][tasks[i]] for i in range(len(tasks))])
            all_assignments.append(score)

        # update values if needed
        all_best_value = max(best_value, all_best_value)  # update the best value if needed
        all_pred_value = max(pred_value, all_pred_value)  # update the predicted value if needed (don't use this metric)
        if adjusted_pred_value > all_adjusted_pred_value:  # update the adjusted predicted value AND the actual value if needed
            all_adjusted_pred_value = adjusted_pred_value
            all_actual_value = actual_value
        all_worst_value = max(worst_value, all_worst_value)  # update the worst value if needed       
        all_expected_values.append(expected_value)

    avg_expected_value = sum(all_expected_values) / len(all_expected_values)  # average the expected values

    return [all_best_value, all_pred_value, all_adjusted_pred_value, all_actual_value, all_worst_value, avg_expected_value, [x for x in all_assignments]]  # record the score data
