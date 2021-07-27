# process_logs.py: converts the log files into a CSV of results

import os

#path = "./logs/"

def get_ni_data(path):
    print("PROCESSING NI DATA")

    csv_lines = []
    ni_scores = {}

    for p in os.listdir(path):
        p = p[:-4]
        with open(path + "/" + p + ".txt", "r") as f:
            ni_scores[p] = 0

            actions = f.readlines()

            for a in range(len(actions)):
                action = actions[a]
                if "networks" in action and "game complete" in action:
                    score = sum([ int(x) for x in action.split(": ")[4].split("[")[1].split("]")[0].split(",") ])
                    ni_scores[p] = score
        
            continue
