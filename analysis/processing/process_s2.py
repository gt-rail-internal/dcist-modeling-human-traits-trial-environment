# determines results for each user

import os
#import networks_analysis

#path = "./logs_stage_3_sa/"

def get_s2_data(path):
    print("PROCESSING S2 DATA")
    s2_scores = {}
    for p in os.listdir(path):
        p = p[:-4]
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
            num_interactions = [0, 0, 0, 0]
            distance_traveled = []

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

                if stage == 2 and "add-valid-waypoint" in a and reset == True:
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

                # get the distance traveled
                if stage == 2 and started == True and "'stage': 2, 'action': 'distance-checkup'" in a:
                    distance_traveled = a.split("'")[9].split(":")[1].split(",")
                    distance_traveled = [int(float(x)) for x in distance_traveled]

                # get the number of connected caches
                if stage == 2 and started == True and "'stage': 2, 'action': 'cache connected count'" in a:
                    max_caches_connected = int(a.split("'")[8].replace(" ", "").split(":")[1][0])

                if stage == 2 and complete == True:
                    response += "\n" + "  Stage 2 duration" + str(end_time - start_time)
                    if reset_time > 0:
                        response += "\n" + "  Stage 2 reset duration" + str(end_time - reset_time)
                        response += "\n" + "  Number of robots interacted with: " + str(sum(robots_interacted))
                    s2_scores[p] = (end_time - start_time)
                    s2_scores[p] = int(100000 * max_caches_connected / sum(distance_traveled))
                    break

                former_a = a

    return s2_scores