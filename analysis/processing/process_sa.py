import os

def get_sa_data(path):
    print("PROCESSING SA DATA")
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
                    sa_scores[p] = round(int(a[:-1].split(", ")[2].split(": ")[1]) / 85, 2)  # 85 is the highest score possible

    return sa_scores
