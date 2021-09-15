# determines results for each user

import os
import math
#import networks_analysis

#path = "./logs_stage_3_sa/"

# euclidean distance formula
def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_s1_data(path):
    print("PROCESSING S1 DATA", path)
    s1_scores = {}
    s1_collected = 0
    s1_total = 0
    for p in os.listdir(path):
        p = p[:-4]
        s1_scores[p] = 0
        if "replay" not in p:
            continue
        print("LOOKING AT USER", p)

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
                    cache_states = [float(x) for x in a.split("'")[17].split(":")[1].split(",")]
                    # calculate the distance to each cache
                    for i in range(len(min_cache_dist)):  # for each cache
                        # for each robot
                        for r in range(len(robot_locations) // 3):
                            # get dist
                            dist = distance(cache_locations[i], [robot_locations[3 * r], robot_locations[3 * r + 1]])
                            # update if less than current recorded distance
                            min_cache_dist[i] = dist if dist < min_cache_dist[i] else min_cache_dist[i]
                    

                # finish the stage
                if stage == 1 and started == True and ("stage-complete" in a or int(a[:10]) - start_time > 60 * 10):
                    complete = True
                    end_time = int(a[:10])
                    print("\n" + "  Number of caches identified: " + str(cache_collected))

                if stage == 1 and complete == True:
                    duration = end_time - start_time
                    print("\n" + "  Stage 1 duration " + str(duration))
                    print("min dist", str(min_cache_dist), "=", sum(min_cache_dist))
                    print("cache states", cache_states)
                    s1_scores[p] = sum(min_cache_dist) / duration
                        
                    break

                former_a = a
        
        s1_collected += 1 if cache_collected == 5 else 0
        s1_total += 1
        s1_ = [s1_scores[p] for p in s1_scores]
        #print("___", s1_)
        #print("S1 Cache", cache_collected, s1_collected, s1_total)

                #if complete:
    return s1_scores

#get_s1_data("logs")