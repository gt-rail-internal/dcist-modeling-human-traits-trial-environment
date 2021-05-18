dists = {
    1: {
        0: 8,
        1: 7,
        2: 6,
        3: 5,
        4: 4,
        5: 3,
        6: 2,
        7: 3,
        8: 2,
        9: 1,
        10: 0,
        11: 1,
        12: 1,
        13: 2,
        14: 3,
        15: 3,
        16: 4,
        17: 5,
        18: 4,
        19: 5,
        20: 5,
        21: 6,
        22: 7,
        23: 8,
        24: 9,
        25: 10,
    },
    2: {
        0: 4,
        1: 3,
        2: 3,
        3: 3,
        4: 4,
        5: 3,
        6: 2,
        7: 1,
        8: 0,
        9: 0,
        10: 1,
        11: 2,
        12: 3,
        13: 4,
        14: 4,
        15: 4,
        16: 4,
        17: 4,
        18: 4,
        19: 3,
        20: 4,
        21: 4,
        22: 5,
        23: 6,
        24: 7,
        25: 7,
        26: 6,
        27: 5,
    },
    3: {
        0: 0,
        1: 1,
        2: 1,
        3: 1,
        4: 2,
        5: 3,
        6: 4,
        7: 4,
        8: 5,
        9: 6,
        10: 7,
        11: 7,
        12: 6,
        13: 5,
        14: 4,
        15: 3,
        16: 2,
        17: 1,
        18: 1,
        19: 1,
        20: 0,
        21: 1,
        22: 1,
        23: 2,
        24: 3,
        25: 4,
    },
    4: {
        0: 6,
        1: 5,
        2: 4,
        3: 3,
        4: 2,
        5: 1,
        6: 0,
        7: 0,
        8: 1,
        9: 2,
        10: 3,
        11: 4,
        12: 5,
    },
    5: {
        0: 6,
        1: 5,
        2: 4,
        3: 3,
        4: 2,
        5: 1,
        6: 0,
        7: 1,
        8: 2,
        9: 3,
        10: 4,
        11: 5,
        12: 6,
    },
}

def determine_score(picks):
    score = 0

    if len(picks) < 16:
        print("  Networks has less than expected picks", len(picks))
        return -1, [], []
    
    if len(picks) > 16:
        print("  Networks has more than expected picks", len(picks))
        return -1, [], []

    p1_attempt1 = int(picks[0])
    p1_attempt2 = int(picks[1])
    p1_attempt3 = int(picks[2])
    p2_attempt1 = int(picks[3])
    p2_attempt2 = int(picks[4])
    p2_attempt3 = int(picks[5])
    easy = int(picks[6])
    p3_attempt1 = int(picks[7])
    p3_attempt2 = int(picks[8])
    p3_attempt3 = int(picks[9])
    p4_attempt1 = int(picks[10])
    p4_attempt2 = int(picks[11])
    p4_attempt3 = int(picks[12])
    p5_attempt1 = int(picks[13])
    p5_attempt2 = int(picks[14])
    p5_attempt3 = int(picks[15])

    if easy != 2:
        print("  Network data may need to be cured")
        return -1, [], []

    raws = [[], [], [], [], []]

    P1 = dists[1][p1_attempt1]
    raws[0].append(dists[1][p1_attempt1])

    P1 = (P1 + dists[1][p1_attempt2]) / 2 if p1_attempt2 != -1 else P1
    if p1_attempt2 == -1:
        raws[0].append(0)
    else:
        raws[0].append(dists[1][p1_attempt2])

    P1 = (2*P1 + dists[1][p1_attempt3]) / 3 if p1_attempt3 != -1 else 2 * P1 / 3  # if the second attempt was right, say the third was right too
    if p1_attempt3 == -1:
        raws[0].append(0)
    else:
        raws[0].append(dists[1][p1_attempt3])
    score += P1
    #print("  P1:", P1)

    P2 = dists[2][p2_attempt1]
    raws[1].append(dists[2][p2_attempt1])

    P2 = (P2 + dists[2][p2_attempt2]) / 2 if p2_attempt2 != -1 else P2
    if p2_attempt2 == -1:
        raws[1].append(0)
    else:
        raws[1].append(dists[2][p2_attempt2])

    P2 = (2*P2 + dists[2][p2_attempt3]) / 3 if p2_attempt3 != -1 else 2 * P2 / 3  # if the second attempt was right, say the third was right too
    if p2_attempt3 == -1:
        raws[1].append(0)
    else:
        raws[1].append(dists[2][p2_attempt3])

    score += P2
    #print("  P2:", P2)

    P3 = dists[3][p3_attempt1]
    P3 = (P3 + dists[3][p3_attempt2]) / 2 if p3_attempt2 != -1 else P3
    if p3_attempt2 == -1:
        raws[2].append(0)
    else:
        raws[2].append(dists[3][p3_attempt2])

    P3 = (2*P3 + dists[3][p3_attempt3]) / 3 if p3_attempt3 != -1 else 2 * P3 / 3  # if the second attempt was right, say the third was right too
    if p3_attempt3 == -1:
        raws[2].append(0)
    else:
        raws[2].append(dists[3][p3_attempt3])

    score += P3
    #print("  P3:", total)

    P4 = dists[4][p4_attempt1]
    P4 = (P4 + dists[4][p4_attempt2]) / 2 if p4_attempt2 != -1 else P4
    if p4_attempt2 == -1:
        raws[3].append(0)
    else:
        raws[3].append(dists[4][p4_attempt2])

    P4 = (2*P4 + dists[4][p4_attempt3]) / 3 if p4_attempt3 != -1 else 2 * P4 / 3  # if the second attempt was right, say the third was right too
    if p4_attempt3 == -1:
        raws[3].append(0)
    else:
        raws[3].append(dists[4][p4_attempt3])

    score += P4
    #print("  P4:", P4)

    P5 = dists[5][p5_attempt1]
    P5 = (P5 + dists[5][p5_attempt2]) / 2 if p5_attempt2 != -1 else P5
    if p5_attempt2 == -1:
        raws[4].append(0)
    else:
        raws[4].append(dists[5][p5_attempt2])

    P5 = (2*P5 + dists[5][p5_attempt3]) / 3 if p5_attempt3 != -1 else 2 * P5 / 3  # if the second attempt was right, say the third was right too
    if p5_attempt3 == -1:
        raws[4].append(0)
    else:
        raws[4].append(dists[5][p5_attempt3])

    score += P5
    #print("  P5:", P5)
    
    return score, [P1, P2, P3, P4, P5], raws


def load_picks():
    picks = {}

    with open("data.csv", "r") as f:
        lines = f.readlines()
    
    for line in lines:
        vals = line.split(",")
        picks[vals[0]] = vals[1:-1]  # cut off the ending newline
    
    return picks


#picks = load_picks()
#for user in picks:
#    score = determine_score(picks[user])
#    print(user, score)