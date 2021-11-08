from statistics import mean

# process the data logs into a user score matrix
#    input: log file path
#    output: dict{dict{}}, N x (T + J) matrix, N is number of participants, T is number of traits, J is number of tasks
def process_logs(path="logs", specific_users=[]):
    # process the text files to get user scores
    import processing.process_ot
    import processing.process_sa
    import processing.process_ni
    import processing.process_s1
    import processing.process_s2
    import processing.process_s3

    ot_data = processing.process_ot.get_ot_data(path, specific_users=[])
    sa_data = processing.process_sa.get_sa_data(path, specific_users=[])
    ni_data = processing.process_ni.get_ni_data(path, specific_users=[])
    s1_data = processing.process_s1.get_s1_data(path, specific_users=[])
    s2_data = processing.process_s2.get_s2_data(path, specific_users=[])
    s3_data = processing.process_s3.get_s3_data(path, specific_users=[])


    # combine into a dictionary format
    user_scores = {}
    for p in {**sa_data, **ni_data, **ot_data, **s1_data, **s2_data, **s3_data}:
        if "email" in p or p == "0" or "none" in p:  # ignore non-user data
            continue
        if p not in user_scores:  # add user to user scores if applicable
            user_scores[p] = {}
        # add scores to the user
        user_scores[p]["sa"] = sa_data[p] if p in sa_data else -1
        user_scores[p]["ni"] = ni_data[p] if p in ni_data else -1
        user_scores[p]["ot"] = ot_data[p] if p in ot_data else -1
        user_scores[p]["s1"] = s1_data[p] if p in s1_data else -1
        user_scores[p]["s2"] = s2_data[p] if p in s2_data else -1
        user_scores[p]["s3"] = s3_data[p] if p in s3_data else -1
    
    return user_scores


import random
# generate random users to test the allocation algorithms
#    input: N (number of users), trait_noise (standard deviation of user trait scores, on the scale of %), task_noise (standard deviation of user task scores, on the scale of %)
#    output: dict{dict{}}, N x (T + J) matrix, N is number of participants, T is number of traits, J is number of tasks
def generate_fake_user_scores(N=30, trait_noise=0, task_noise=0):
    user_scores = {}

    tasks = ["s1", "s2", "s3"]
    traits = ["ot", "ni", "sa"]

    # generate the slopes for each trait/task relationship
    slopes = {}
    y_ints = {}
    for task in tasks:
        slopes[task] = {}
        y_ints[task] = {}
        for trait in traits:
            slopes[task][trait] = random.random() / 2  # scale from 0 to .5
            y_ints[task][trait] = random.gauss(.5, .2)
            y_ints[task][trait] -= max(0, slopes[task][trait] + y_ints[task][trait] - 1)  # correct the y intercept so no scores are above 1

    # for each user, generate fake scores
    for i in range(N):
        # generate the user ID, ensure it's not already used
        while True:
            p = random.randint(1000, 9999)  # user ID
            if p not in user_scores:
                break
        
        user_scores[p] = {}

        # for each task
        for t in range(len(tasks)):
            # generate the trait score
            user_scores[p][traits[t]] = 1.1
            while user_scores[p][traits[t]] > 1 or user_scores[p][traits[t]] < 0:
                user_scores[p][traits[t]] = random.gauss(.5, trait_noise)

            # generate diagonal task scores
            user_scores[p][tasks[t]] = 1.1
            while user_scores[p][tasks[t]] > 1 or user_scores[p][tasks[t]] < 0:
                user_scores[p][tasks[t]] = slopes[tasks[t]][traits[t]] * user_scores[p][traits[t]] + (2 * random.random() - 1) * task_noise + y_ints[tasks[t]][traits[t]]  # theoretical + noise + y_int

    p = list(user_scores.keys())[0]
    
    # return the user scores
    return user_scores, slopes


# pulls every human subset
#   input: N (number of rows), J (number of assignments))
#   output: subsets (N^(J-1)xJ)
import itertools
def pullSubsets(N, J, subsets=[], subset=[], slot=-1, all=False):
    if not all:
        subsets = [list(x) for x in itertools.combinations(range(N), J)]
    if all:
        subsets = [list(x) for x in itertools.permutations(range(N), J)]
    return subsets


# function for calculating best fit line, from (https://stackoverflow.com/questions/10048571)
def best_fit_slope(score_pairing):
    xs = [x[0] for x in score_pairing]
    ys = [x[1] for x in score_pairing]
    N = len(xs)
    Sx = Sy = Sxx = Syy = Sxy = 0.0
    for x, y in zip(xs, ys):
        Sx = Sx + x
        Sy = Sy + y
        Sxx = Sxx + x*x
        Syy = Syy + y*y
        Sxy = Sxy + x*y
    det = Sxx * N - Sx * Sx
    return (Sxy * N - Sy * Sx)/det, (Sxx * Sy - Sx * Sxy)/det
    
# function for removing outliers (1.5x IQR)
def remove_outliers(scores):
    # if empty list, return
    if len(scores) == 0:
        return []
    
    # remove the outlier scores for each dimension
    for index in range(len(scores[0])):
        Q1 = sorted([x[index] for x in scores])[len(scores) // 4]
        Q3 = sorted([x[index] for x in scores])[int(len(scores) // 1.25)]
        IQR = Q3 - Q1
        outlier_min = Q1 - IQR * 1.5
        outlier_max = Q3 + IQR * 1.5
        scores = [x for x in scores if x[index] > outlier_min and x[index] < outlier_max]
    return scores


# function for generating the impact matrix from user scores
import scipy.stats
def generate_impact_matrix(user_scores, traits, tasks):
    impact_matrix = {}
    yint_matrix = {}  # while not necessary, including the y intercept matrix so we can play with absolute predicted scores (not just relative scores)
    weight_matrix = {}

    # generate the impact matrix: slope for each trait/task pairing
    for trait in traits:
        for task in tasks:
            # get the trait/task scores
            pairing = [[user_scores[p][trait], user_scores[p][task]] for p in user_scores if user_scores[p][trait] != -1 and user_scores[p][task] not in [-1, 0]]

            # remove outliers
            pairing = remove_outliers(pairing)

            # determine the slopes of each best fit line
            m, y = best_fit_slope(pairing)
            
            # store the slopes in the impact matrix
            if trait not in impact_matrix:
                impact_matrix[trait] = {}
                yint_matrix[trait] = {}
                weight_matrix[trait] = {}
            impact_matrix[trait][task] = m
            yint_matrix[trait][task] = y
            weight_matrix[trait][task] = scipy.stats.spearmanr([x[0] for x in pairing], [x[1] for x in pairing])[0]
        
        # normalize the weight matrix
        for task in tasks:
            weight_matrix[trait][task] /= sum([abs(x) for x in weight_matrix[trait].values()])
    
    return impact_matrix, yint_matrix, weight_matrix


# function for predicting a user's score given the impact matrix and the user test scores
def predict_test_user_performance(test_user_scores, impact_matrix={}, yint_matrix={}, weight_matrix={}, traits=[], tasks=[]):
    score_predictions = {p : {} for p in test_user_scores}
    for p in test_user_scores:
        for task in tasks:
            if yint_matrix == {}:
                score_predictions[p][task] = sum([weight_matrix[trait][task] * impact_matrix[trait][task] * test_user_scores[p][trait] for trait in traits])  # sum of weight * slope * trait
            else:
                score_predictions[p][task] = sum([weight_matrix[trait][task] * (impact_matrix[trait][task] * test_user_scores[p][trait] + yint_matrix[trait][task]) for trait in traits])
        # if some stages are ignored, fill their values with 0
        if "s1" not in tasks:
            score_predictions[p]["s1"] = 0
        if "s2" not in tasks:
            score_predictions[p]["s2"] = 0
        if "s3" not in tasks:
            score_predictions[p]["s3"] = 0
    return score_predictions


# optimally assigns each human worker to each job 
#   input: S (NxJ matrix of human job cost)
#   output: bool (whether a solution was found), a (job assignment of each worker)
from munkres import Munkres
def hungarian(S, maximize=True):
    # if maximizing the score, multiply everything by -1 (finding min of negation finds the max)
    if maximize:
        S_final = [[x * -1 for x in y] for y in S]
    else:
        S_final = S

    # relies on the Munkres library (via Pip)
    m = Munkres()
    indexes = m.compute(S_final)
    
    a = [col for row, col in indexes]  # [user1 task idx, user2 task idx, user3 task idx]
    return a


# function for filtering out users who do not have complete data
def filter_complete_users(user_scores, traits=[], tasks=[]):
    # ignores users with a -1 trait (incomplete data), -1 task (incomplete data), or a 0 task (did not complete any base objectives)
    filtered_scores = {p : user_scores[p] for p in user_scores if -1 not in [user_scores[p][trait] for trait in traits] and -1 not in [user_scores[p][task] for task in tasks] and 0 not in [user_scores[p][task] for task in tasks]}
    return filtered_scores


# processes a set of user scores into the trait-based team assignments for all 3-user teams, onehot
import allocation.onehot_allocation
import random
def process_users(user_scores, complete_user_scores, traits=[], tasks=[]):
    # define the trait and tasks
    prediction_tasks = tasks

    # generate the one hot of test and trait data
    num_train = len(user_scores) - len(tasks)  # the number of items to train on
    num_test = len(tasks)  # the number of items to test on

    complete_user_scores_ids = list(complete_user_scores.keys())  # pull the IDs of the users who completed all stages
    team_indexes = pullSubsets(len(complete_user_scores_ids), len(tasks))  # pull each possible team, in the form of indexes instead of IDs

    # train and evaluate on each fold
    score_data = []  # holds the data for each team combination
    for team in team_indexes:
        # extract the test/train user IDs from the team indexes
        test_ids = [complete_user_scores_ids[i] for i in team]  # convert the team indexes to user IDs
        score_data.append(allocation.onehot_allocation.onehot_allocation(complete_user_scores, test_ids, traits, tasks, prediction_tasks))  # record the score data
    
    return score_data