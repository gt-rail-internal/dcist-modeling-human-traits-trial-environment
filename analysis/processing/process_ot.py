import os

def get_ot_data(path, specific_users=[]):
    #print("PROCESSING OT DATA")
    ot_scores = {}
    seen = {}
    vals = {}
    for p in os.listdir(path):
        # ignore replays
        if "replay" in p:
            continue

        # check specific users
        if specific_users != [] and sum([1 for x in specific_users if x in p]) == 0:
            continue

        p = p[:-4]
        ot_scores[p] = -1
        with open(path + "/" + p + ".txt", "r") as f:
            actions = f.readlines()

            for i in range(len(actions)):
                a = actions[i]

                if "Howfamiliarareyouwithrobots" in a and p not in seen:
                    print(">>>", p, a.split(", ")[2])
                    seen[p] = a.split(", ")[2][-1]
                    if a.split(", ")[2][-1] not in vals:
                        vals[a.split(", ")[2][-1]] = 0
                    vals[a.split(", ")[2][-1]] += 1

                # if completed the SAGAT, print the score
                if "object tracking" in a and "result" in a:
                    ot_scores[p] = round(sum([int(x) for x in a[:-1].split("[")[1].split("]")[0].split(",")][1:]) / 27, 2)  # 27 is the highest score possible (2+3+4+5+6+7), ignoring first round
                    
    print(seen)
    print(vals)
    return ot_scores

get_ot_data("logs")