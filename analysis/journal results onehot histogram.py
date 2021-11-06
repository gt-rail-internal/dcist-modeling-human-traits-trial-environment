# analyzes the data to determine the results

from statistics import median
import allocation.assignment_util

# whether we are plotting or training
PLOTTING = True

# check if we have a results, if so display them
if PLOTTING:
    import sys, os.path, pickle
    # check if the file exists
    if not os.path.isfile("f3c3fake_results.pkl"):
        print("No results file")
        sys.exit()

    # load the list
    with open("f3c3fake_results.pkl", "rb") as f:
        iteration_scores = pickle.load(f)
    
    # format data into histogram distributions
    from matplotlib import pyplot as plt
    import itertools
    data = [[x[i] for x in iteration_scores] for i in range(len(iteration_scores[0])-1)]  # -1 to not consider the last (all possibilities)
    all_assignments = list(itertools.chain.from_iterable([x[-1] for x in iteration_scores]))
    print(len(data[-1]))

    # as an aside, determine how trait-based compares to all-assignments within a 3-user group
    tb_wins = 0  # count of trait-based wins
    all_wins = 0  # count of all-assignment wins
    tie = 0  # count of ties
    todos = [x[-1] for x in iteration_scores]  # all possible assignments for each set of 3 users (6)
    for d in range(len(data[0])):  # for each set of 3 users
        tb = data[3][d]  # pull their trait-based score
        for all in todos[d]:  # for each possible score for that set
            if tb > all:  # if trait-based wins
                tb_wins += 1
            if tb == all:  # if they tie (should be >= num of 3 user sets, 3276)
                tie += 1
            if tb < all:  # if all-assignment wins
                all_wins += 1

    print("TB", tb_wins, "ALL", all_wins, "TIE", tie)
    print("m1", sum(all_assignments) / len(all_assignments), "m2", sum(data[3]) / len(data[3]))
    ax = plt.gca()  # get the plot axis
    bins = 50
    background_alpha = .4
    alpha = 0.5
    range = [0, 3]
    
    #print(all_assignments)
    import numpy
    (counts, bins) = numpy.histogram(all_assignments, bins=bins)
    plot = ax.hist(data[4], bins=bins, range=range, alpha=alpha, color="red", label="Known Worst")  # histogram for worst scores
    plot = ax.hist(data[0], bins=bins, range=range, alpha=alpha, color="green", label="Known Best")  # histogram for best scores
    plot = ax.hist(bins[:-1], bins, weights=counts / 6, range=range, alpha=alpha, color="orange", label="All Possibilities")
    plot = ax.hist(data[3], bins=bins, range=range, alpha=alpha, color="blue", label="Trait-Based")  # histogram for predicted actual

    ax.legend()
    ax.set_title("[Assigning 3 Contrived Users] Histogram of Scores via Assignment Methods with all Traits Applied, N=" + str(len(data[0])))
    ax.set_xlabel("Team Assignment Score")  # set x label
    ax.set_xlim([0, 3])  # set locations of x ticks
    ax.set_ylabel("Number of Teams")  # set y label
    plt.show()

else:
    # define the trait and tasks
    traits=["ot", "sa", "ni"]
    tasks=["s1", "s2", "s3"]
    prediction_tasks = tasks

    # process the data into user scores matrix
    specific_users = []
    #user_scores = allocation.assignment_util.process_logs("logs", specific_users=specific_users)  # all user scores (even those who failed to have complete data)
    user_scores = allocation.assignment_util.generate_fake_user_scores(N=30, trait_noise=0.2, task_noise=0)
    complete_user_scores = allocation.assignment_util.filter_complete_users(user_scores, traits=traits, tasks=tasks)  # ONLY users who have complete data

    # generate the one hot of test and trait data
    import random
    import onehot_allocation

    num_train = len(complete_user_scores) - len(tasks)  # the number of items to train on
    num_test = len(tasks)  # the number of items to test on

    complete_user_scores_ids = list(complete_user_scores.keys())  # pull the IDs of the users who completed all stages
    team_indexes = allocation.assignment_util.pullSubsets(len(complete_user_scores_ids), len(tasks))  # pull each possible team, in the form of indexes instead of IDs

    # train and evaluate on each fold
    score_data = []  # holds the data for each team combination
    for team in team_indexes:
        # extract the test/train user IDs from the team indexes
        test_ids = [complete_user_scores_ids[i] for i in team]  # convert the team indexes to user IDs
        score_data.append(onehot_allocation.onehot_allocation(user_scores, test_ids, traits, tasks, prediction_tasks))  # record the score data

    # save the iteration scores
    import pickle

    with open("f3c3fake_results.pkl", "wb") as f:
        pickle.dump(score_data, f)

