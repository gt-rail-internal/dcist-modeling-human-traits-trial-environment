import matplotlib.pyplot as plt
import itertools
import random

# generate users
users = {}
user_ids = ["AA", "BB", "CC", "DD", "EE", "FF", "GG", "HH", "II", "JJ"]
for user_id in user_ids:
    users[user_id] = {"M1": random.random(), "M2": random.random(), "M3": random.random()}
users["KK"] = users["AA"]
users["LL"] = users["AA"]

print("Users", users)

team_size = 3
tasks = ["M1", "M2", "M3"]

scores = {}

# for each sample size (plots)
for sample_size in [8, 7, 6, 5, 4, 3]:
    scores[sample_size] = []

    # sample each combination (no repeated combos, no repeated users)
    samples = [list(x) for x in itertools.combinations(users.keys(), sample_size)]
    print("Number of samples:", len(samples), "example:", samples[0])

    # for each sample in the samples, generate teams and all possible assignments
    for sample in samples:
        # generate teams
        teams = [list(x) for x in itertools.combinations(sample, team_size)]

        # for each team in teams, generate all possible assignments
        for team in teams:
            # generate assignments
            assignments = [list(x) for x in itertools.permutations(team, len(tasks))]

            # for each assignment, calculate score
            for assignment in assignments:
                score = sum([users[assignment[i]][tasks[i]] for i in range(len(tasks))])
                scores[sample_size].append(score)

    # plot the resulting histograms
    plt.hist(x=scores[sample_size], bins=20, alpha=0.4)

    # generate stats
    print("  Stats", "mean", sum(scores[sample_size]) / len(scores[sample_size]))

plt.show()
