import os

def get_ot_data(path):
    print("PROCESSING OT DATA")
    ot_scores = {}
    for p in os.listdir(path):
        p = p[:-4]
        ot_scores[p] = -1
        with open(path + "/" + p + ".txt", "r") as f:
            actions = f.readlines()

            for i in range(len(actions)):
                a = actions[i]

                # if completed the SAGAT, print the score
                if "object tracking" in a and "result" in a:
                    ot_scores[p] = round(sum([int(x) for x in a[:-1].split("[")[1].split("]")[0].split(",")][1:]) / 27, 2)  # 27 is the highest score possible (2+3+4+5+6+7), ignoring first round
                    #print(p + ",", ot_scores[p])
    return ot_scores

