# determines results for each user

import os
import math
#import networks_analysis

#path = "./logs_stage_3_sa/"

# euclidean distance formula
def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_s1_data(path, specific_users=[], metric="distance progress"):
    #print("PROCESSING S1 DATA")
    s1_scores = {}
    s1_collected = 0
    s1_total = 0
    for p in os.listdir(path):
        p = p[:-4]

        if "replay" in p and "S1" not in p:
            continue

        # check specific users
        if specific_users != [] and sum([1 for x in specific_users if x in p]) == 0:
            continue

        #print(">", p)

        worker_id = p.split("-")[0]
        if not worker_id.isnumeric():
            continue 
        
        if worker_id not in s1_scores:
            s1_scores[worker_id] = 1
        #print("LOOKING AT USER", p)

        with open(path + "/" + p + ".txt", "r") as f:
            #print("S1 Looking at user" + " " + p)

            actions = f.readlines()

            started = False
            start_time = 0

            reset = False
            reset_time = 0
            
            complete = False
            end_time = 0

            robots_interacted = [0, 0, 0, 0, 0, 0, 0, 0]
            robot_locations = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            num_interactions = [0, 0, 0, 0]
            distance_traveled = []

            cache_collected = 0
            cache_returned = 0
            cache_identified = 0
            cache_states = [0, 0, 0, 0, 0]
            cache_locations = [[111, 273], [372, 356], [640, 150], [607, 523], [109, 578]]
            min_cache_dist = [float("inf"), float("inf"), float("inf"), float("inf"), float("inf")]

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

                # if completed Stage 1, get stats
                if "'stage': 1" in a and stage == 0:
                    stage = 1

                # start when a robot is no longer idle
                if stage == 1 and "no longer idle" in a and not started:
                    started = True
                    start_time = int(a[:10])

                if stage == 1 and "add-valid-waypoint" in a and not started:
                    started = True
                    start_time = int(a[:10])
                    response += "\n" + "  Stage 1 Start " + a[:11]

                if stage == 1 and "collected cache" in a:
                    cache_collected += 1

                # get the distance traveled
                if stage == 1 and started == True and "'stage': 1, 'action': 'distance-checkup'" in a and not complete:
                    distance_traveled = a.split("'")[9].split(":")[1].split(",")

                # get the robot and cache states
                if stage == 1 and started == True and "locations" in a and not complete:
                    robot_locations = [float(x) for x in a.split("'")[13].split(":")[1].split(",")]
                    if len(a.split("'")) > 17 and len(a.split("'")[17].split(":")) > 1:
                        cache_states = [float(x) for x in a.split("'")[17].split(":")[1].split(",")]
                    # calculate the distance to each cache
                    for ii in range(len(min_cache_dist)):  # for each cache
                        # for each robot
                        for r in range(len(robot_locations) // 3):
                            # get dist
                            dist = distance(cache_locations[ii], [robot_locations[3 * r], robot_locations[3 * r + 1]])
                            # update if less than current recorded distance
                            min_cache_dist[ii] = dist if dist < min_cache_dist[ii] else min_cache_dist[ii]

                # finish the stage
                if stage == 1 and started == True and ("stage-complete" in a or int(a[:10]) - start_time > 60 * 10 or i == len(actions)-1):
                    complete = True
                    end_time = int(a[:10])
                    #print("  Number of caches identified: " + str(cache_collected))

                if stage == 1 and complete == True:
                    duration = end_time - start_time
                    
                    # the initial cache locations
                    initial_dist = sum([294.7032405658275, 329.2152862712491, 581.7380457581583, 255.74581870331477, 314.70295872494506])

                    # set the user score
                    if metric == "distance progress":
                        # if this run is a replay and the current score value is not 1, override
                        if "replay" in p and s1_scores[worker_id] == 1 / 600:
                            s1_scores[worker_id] = (initial_dist - sum(min_cache_dist)) / 600

                        # if this run is a replay and the value is still 1, set the numerator
                        elif "replay" in p and s1_scores[worker_id] == 1:
                            s1_scores[worker_id] *= (initial_dist - sum(min_cache_dist))
                        
                        # if this run is NOT a replay and the duration is less than 600, override
                        elif "replay" not in p and duration < 600:
                            s1_scores[worker_id] = initial_dist / duration
                        
                        # if this run is NOT A replay and the duration >= 600, divide by 600
                        elif "replay" not in p and duration >= 600:
                            s1_scores[worker_id] /= 600

                    elif metric == "number of caches located":
                        s1_scores[worker_id] = cache_collected

                    else:
                        s1_scores[worker_id] = -1

                    #print("done with", p)
                    break

                former_a = a

    # normalize scores
    normalization = max(s1_scores.values())  # normalization factor

    s1_scores = {p : s1_scores[p] / normalization for p in s1_scores}
    
    s1_collected += 1 if cache_collected == 5 else 0
    s1_total += 1

    return s1_scores