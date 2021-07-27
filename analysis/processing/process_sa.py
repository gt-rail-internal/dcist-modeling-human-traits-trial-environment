import os

def get_sa_data(path):
    sa_scores = {}
    for p in os.listdir(path):
        p = p[:-4]
        sa_scores[p] = -1
        with open(path + "/" + p + ".txt", "r") as f:
            actions = f.readlines()

            for i in range(len(actions)):
                a = actions[i]

                # if completed the SAGAT, print the score
                if "SAGAT" in a and "'complete'" in a:
                    sa_scores[p] = int(a[:-1].split(", ")[2].split(": ")[1])

    print("sa", sa_scores)
    return sa_scores
