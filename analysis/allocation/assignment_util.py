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


# pulls every human subset
#   input: N (number of rows), J (number of assignments))
#   output: subsets (N^(J-1)xJ)
import itertools
def pullSubsets(N, J, subsets=[], subset=[], slot=-1, all=False):
    subsets = [list(x) for x in itertools.combinations(range(N), J)]
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
def generate_impact_matrix(user_scores, traits, tasks):
    impact_matrix = {}
    yint_matrix = {}  # while not necessary, including the y intercept matrix so we can play with absolute predicted scores (not just relative scores)
    
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
            impact_matrix[trait][task] = m
            yint_matrix[trait][task] = y
        
    return impact_matrix, yint_matrix


# function for predicting a user's score given the impact matrix and the user test scores
def predict_test_user_performance(test_user_scores, impact_matrix={}, yint_matrix={}, traits=[], tasks=[]):
    score_predictions = {p : {} for p in test_user_scores}
    for p in test_user_scores:
        for task in tasks:
            #print("-", [(yint_matrix[trait][task] if yint_matrix != {} else 0) for trait in traits])
            if yint_matrix == {}:
                score_predictions[p][task] = sum([impact_matrix[trait][task] * test_user_scores[p][trait] for trait in traits])
            else:
                score_predictions[p][task] = sum([(impact_matrix[trait][task] * test_user_scores[p][trait] + yint_matrix[trait][task]) / len(traits) for trait in traits])
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
