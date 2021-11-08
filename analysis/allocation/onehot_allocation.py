import allocation.assignment_util

# runs onehot allocation on a set of test users
def onehot_allocation(user_scores, test_ids, traits, tasks, prediction_tasks=[]):
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

    # use the impact matrix to get the predicted score for each of the test users
    score_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix={}, weight_matrix=weight_matrix, traits=traits, tasks=prediction_tasks)  # predicted user scores for each task
    score_adjusted_prediction_matrix = allocation.assignment_util.predict_test_user_performance(test_user_scores, impact_matrix=impact_matrix, yint_matrix=yint_matrix, weight_matrix=weight_matrix, traits=traits, tasks=tasks)  # predicted user scores for each task INCLUDING Y INTERCEPT (not used by algorithm)
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
    for i in range(len(test_ids)):
        idx = test_ids[i]
        best_value += user_scores[idx][tasks[best_a[i]]]  # add the actual score to the value of that team assignment
        pred_value += pred[i][pred_a[i]] # add the predicted score to the value of that team assignment
        adjusted_pred_value += adjusted_pred[i][pred_a[i]]  # add the adjusted predicted value (+y intercepts)
        actual_value += user_scores[idx][tasks[pred_a[i]]]  # add the predicted score to the value of that team assignment
        worst_value += user_scores[idx][tasks[worst_a[i]]]  # add the worst score to the value of that team assignment

    # get the expected value for the team
    expected_value = 0
    all_assignments = []
    score = user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[2]][tasks[2]]
    expected_value += score
    all_assignments.append(score)
    score = user_scores[test_ids[0]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[1]][tasks[2]]
    expected_value += score
    all_assignments.append(score)
    score = user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[2]][tasks[1]] + user_scores[test_ids[0]][tasks[2]]
    expected_value += score
    all_assignments.append(score)
    score = user_scores[test_ids[1]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[2]][tasks[2]]
    expected_value += score
    all_assignments.append(score)
    score = user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[0]][tasks[1]] + user_scores[test_ids[1]][tasks[2]]
    expected_value += score
    all_assignments.append(score)
    score = user_scores[test_ids[2]][tasks[0]] + user_scores[test_ids[1]][tasks[1]] + user_scores[test_ids[0]][tasks[2]]
    expected_value += score
    all_assignments.append(score)
    expected_value /= 6

    return [best_value, pred_value, adjusted_pred_value, actual_value, worst_value, expected_value, [x for x in all_assignments]]  # record the score data
