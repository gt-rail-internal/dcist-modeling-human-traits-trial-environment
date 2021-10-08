# process_logs.py: converts the log files into a CSV of results

import os

#path = "./logs/"

def get_ni_data(path, specific_users=[]):
    #print("PROCESSING NI DATA")

    csv_lines = []
    ni_scores = {}

    for p in os.listdir(path):
        # ignore replays
        if "replay" in p:
            continue

        # check specific users
        if specific_users != [] and sum([1 for x in specific_users if x in p]) == 0:
            continue

        p = p[:-4]
        with open(path + "/" + p + ".txt", "r") as f:
            ni_scores[p] = 0

            actions = f.readlines()

            for a in range(len(actions)):
                action = actions[a]
                if "networks" in action and "game complete" in action:
                    score = round(1 - sum([ int(x) for x in action.split(": ")[4].split("[")[1].split("]")[0].split(",") ]) / 70, 2)  # using 70 as a max (bad) score, most players cap at 60, 1-score so higher is better
                    ni_scores[p] = score
                    #print(ni_scores[p], p)
            continue
    return ni_scores