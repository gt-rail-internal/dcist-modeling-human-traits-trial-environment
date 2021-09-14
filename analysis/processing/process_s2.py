# determines results for each user

import os
import math
#import networks_analysis

#path = "./logs_stage_3_sa/"

# euclidean distance formula
def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_s2_data(path):
    print("PROCESSING S2 DATA")
    s2_scores = {}
    s2_total = 0
    s2_complete = 0
    for p in os.listdir(path):
        p = p[:-4]
        if "replay" not in p or "S2" not in p or "copy" not in p:
            continue
        s2_scores[p] = -1
        with open(path + "/" + p + ".txt", "r") as f:
            #print("S2 Looking at user" + " " + p)

            actions = f.readlines()

            started = False
            start_time = 0

            reset = False
            reset_time = 0
            
            complete = False
            end_time = 0

            robots_interacted = [0, 0, 0, 0, 0, 0, 0, 0]
            robot_locations = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            cache_locations = [[111, 273], [372, 356], [640, 150], [607, 523], [109, 578]]
            num_interactions = [0, 0, 0, 0]
            distance_traveled = []
            min_connection_distance = float("inf")

            max_caches_connected = 0

            cache_collected = 0
            cache_returned = 0
            cache_identified = 0

            last_action_time = 0
            last_action = ""

            stage = 0

            former_a = ""

            response = ""

            for i in range(len(actions)):
                a = actions[i]

                # check for timeout
                this_action_time = int(a[:10]) if isinstance(a[:10], int) and "\n" not in a[:10] else -1
                if i > 0 and this_action_time != -1 and stage == 2 and this_action_time - last_action_time > 60:
                    print("  Idle from action", last_action, "UNTIL", a, "SUM", this_action_time - last_action_time)
                last_action_time = this_action_time
                last_action = a

                # if completed Stage 2, print the start time, reset times, end time, and the duration
                if "stage" in a and a[-3] == "2" and stage == 0:
                    stage = 2
                
                if stage == 2 and "select-vehicle" in a and not started:
                    started = True
                    start_time = int(actions[i-1][:10])
                    response += "\n" + "  Stage 2 Start " + a[:11]

                if stage == 2 and "add-valid-waypoint" in a:
                    if not started:
                        started = True
                        start_time = int(actions[i-1][:10]) if not started else start_time
                    if reset:
                        reset = False
                        reset_time = int(a[:10])
                        response += "\n" + "    Stage 2 reset start " + a[:11]
                    
                if stage == 2 and "add-valid-waypoint" in a:
                    if "UGV1" in a.replace(" ", ""):
                        robots_interacted[0] = 1
                    if "UGV2" in a.replace(" ", ""):
                        robots_interacted[1] = 1
                    if "UGV3" in a.replace(" ", ""):
                        robots_interacted[2] = 1
                    if "UGV4" in a.replace(" ", ""):
                        robots_interacted[3] = 1
                    if "UAV1" in a.replace(" ", ""):
                        robots_interacted[4] = 1
                    if "UAV2" in a.replace(" ", ""):
                        robots_interacted[5] = 1
                    if "UAV3" in a.replace(" ", ""):
                        robots_interacted[6] = 1
                    if "UAV4" in a.replace(" ", ""):
                        robots_interacted[7] = 1

                if "'stage': 2, 'action': 'reset map'" in a:
                    reset = True
                    response += "\n" + "  Stage 2 reset!"

                if stage == 2 and started == True and "'stage': 2, 'action': 'stage-complete'" in a:
                    complete = True
                    end_time = int(a[:10])
                    response += "\n" + "  Stage 2 End " + a[:11]

                    # 406,690.9 535,250 97,461 107,525 150,282 265,560 112,446 619,619

                # get the distance traveled
                if stage == 2 and started == True and "'stage': 2, 'action': 'distance-checkup'" in a:
                    distance_traveled = a.split("'")[9].split(":")[1].split(",")
                    distance_traveled = [int(float(x)) for x in distance_traveled]

                 # get the robot and cache states
                if stage == 2 and started == True and "locations" in a and not complete:
                    robot_locations = [float(x) for x in a.split("'")[13].split(":")[1].split(",")]
                    print(">>>>", [round(x,1) for x in robot_locations])
                    cache_states = [float(x) for x in a.split("'")[17].split(":")[1].split(",")]
                    trial_min_connection_dists = [float("inf"), float("inf"), float("inf"), float("inf"), float("inf")]
                    # calculate the distance to each cache
                    for i in range(5):  # for each cache
                        # for each robot
                        for r in range(8):
                            # get dist to the cache
                            dist = distance(cache_locations[i], [robot_locations[3 * r], robot_locations[3 * r + 1]])
                            # subtract the cache connection dist (UGV: 100, UAV: 75)
                            dist -= 100 if r <= 3 else 75  # if r <= 3 (first four robtos) it's a UGV, otherwise it's a UAV
                            #print("DIST", i, r, dist)
                            # update if less than current recorded distance
                            trial_min_connection_dists[i] = min(max(dist, 0), trial_min_connection_dists[i])

                    print("DIST", p, trial_min_connection_dists)
                    min_connection_distance = min(sum(trial_min_connection_dists), min_connection_distance)

                # get the number of connected caches
                if stage == 2 and started == True and "'stage': 2, 'action': 'cache connected count'" in a:
                    max_caches_connected = int(a.split("'")[8].replace(" ", "").split(":")[1][0])

                if stage == 2 and complete == True:
                    response += "\n" + "  Stage 2 duration" + str(end_time - start_time)
                    if reset_time > 0:
                        response += "\n" + "  Stage 2 reset duration" + str(end_time - reset_time)
                        response += "\n" + "  Number of robots interacted with: " + str(sum(robots_interacted))
                    #s2_scores[p] = (end_time - start_time)
                    #s2_scores[p] = (end_time - start_time) / max_caches_connected if max_caches_connected > 0 else 700 #int(100000 * max_caches_connected / sum(distance_traveled)) if sum(distance_traveled) > 10 else 0
                    s2_scores[p] = min_connection_distance
                    break

                former_a = a

        print("S2", min_connection_distance)
        s2_total += 1
        s2_complete += 1 if max_caches_connected < 5 and max_caches_connected > 0 else 0
        input()
    
    print("S2 Summary", s2_complete, s2_total)

    return s2_scores

get_s2_data("logs")