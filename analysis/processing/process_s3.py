# determines results for each user

import os
#import networks_analysis

#path = "./logs_stage_3_sa/"

def get_s3_data(path):
    print("PROCESSING S3 DATA")
    s3_scores = {}
    for p in os.listdir(path):
        p = p[:-4]
        s3_scores[p] = -1
        with open(path + "/" + p + ".txt", "r") as f:
            #print("S3 Looking at user" + " " + p)

            actions = f.readlines()

            started = False
            start_time = 0

            reset = False
            reset_time = 0
            
            complete = False
            end_time = 0

            robots_interacted = [0, 0, 0, 0, 0, 0, 0, 0]
            num_interactions = [0, 0, 0, 0]
            distance_traveled = []

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

                # if completed Stage 3, get stats
                if "stage" in a and a[-3] == "3" and stage == 0:
                    stage = 3

                if stage == 3 and "add-valid-waypoint" in a and not started:
                    started = True
                    response += "\n" + "  Stage 3 Start"

                if stage == 3 and "distance-traveled" in a:
                    distance_traveled = a.split("'")[9].split(":")[1].split(",")
                
                if stage == 3 and "add-valid-waypoint" in a:
                    if "UGV1" in a:
                        robots_interacted[0] = 1
                    if "UGV2" in a:
                        robots_interacted[1] = 1
                    if "UGV3" in a:
                        robots_interacted[2] = 1
                    if "UGV4" in a:
                        robots_interacted[3] = 1

                if stage == 3 and "UGV1" in a and "select-vehicle" in former_a:
                    num_interactions[0] += 1
                if stage == 3 and "UGV2" in a and "select-vehicle" in former_a:
                    num_interactions[1] += 1
                if stage == 3 and "UGV3" in a and "select-vehicle" in former_a:
                    num_interactions[2] += 1
                if stage == 3 and "UGV4" in a and "select-vehicle" in former_a:
                    num_interactions[3] += 1

                if stage == 3 and "cache collected" in a:
                    cache_collected += 1
                if stage == 3 and "cache returned" in a:
                    cache_returned += 1
                    #response += "\n" + "  Stage 3 Cache Returned"
                
                if stage == 3 and started == True and "stage-complete" in a:
                    complete = True
                    response += "\n" + "  Number of caches collected: " + str(cache_collected)
                    response += "\n" + "  Number of caches returned: " + str(cache_returned)
                    response += "\n" + "  Number of robots interacted with: " + str(sum(robots_interacted))
                    response += "\n" + "  Total robot interactions: " + str(sum(num_interactions))
                    response += "\n" + "  Interactions by robot: " + str(num_interactions[0]) + ", " + str(num_interactions[1]) + ", " + str(num_interactions[2]) + ", " + str(num_interactions[3])
                    response += "\n" + "  Distance Traveled: " + str(sum([float(x) for x in distance_traveled])) + " " + str(distance_traveled)
                    response += "\n" + "  Stage 3 End"
                    
                    # if the user took this stage, score it, otherwise default to -1
                    if sum([float(x) for x in distance_traveled]) > 0:
                        s3_scores[p] = int(100000 * (cache_collected + cache_returned) / sum([float(x) for x in distance_traveled]))
                    else:
                        s3_scores[p] = 0
                    

                former_a = a
                
                #if complete:
    return s3_scores