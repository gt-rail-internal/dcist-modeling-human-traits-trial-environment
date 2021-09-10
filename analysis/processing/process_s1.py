# determines results for each user

import os
#import networks_analysis

#path = "./logs_stage_3_sa/"

def get_s1_data(path):
    print("PROCESSING S1 DATA", path)
    s1_scores = {}
    s1_collected = 0
    s1_total = 0
    for p in os.listdir(path):
        p = p[:-4]
        s1_scores[p] = 0
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

                # if completed Stage 1, get stats
                if "stage" in a and a[-3] == "1" and stage == 0:
                    stage = 1

                if stage == 1 and "add-valid-waypoint" in a and not started:
                    started = True
                    start_time = int(a[:10])
                    response += "\n" + "  Stage 1 Start " + a[:11]

                if stage == 1 and "collected cache" in a:
                    cache_collected += 1

                # get the distance traveled
                if stage == 1 and started == True and "'stage': 1, 'action': 'distance-checkup'" in a and not complete:
                    distance_traveled = a.split("'")[9].split(":")[1].split(",")
                
                if stage == 1 and started == True and "stage-complete" in a:
                    complete = True
                    end_time = int(a[:10])
                    response += "\n" + "  Number of caches identified: " + str(cache_collected)
                    response += "\n" + "  Stage 1 End " + a[:11]

                if stage == 1 and complete == True:
                    response += "\n" + "  Stage 1 duration " + str(end_time - start_time)
                    if cache_collected > 0: #sum([float(x) for x in distance_traveled]) > 0:
                        s1_scores[p] = (end_time - start_time) #int(100000 * cache_collected / sum([float(x) for x in distance_traveled]))
                        
                    else:
                        s1_scores[p] = 0

                    break

                former_a = a
        
        s1_collected += 1 if cache_collected == 5 else 0
        s1_total += 1
        s1_ = [s1_scores[p] for p in s1_scores]
        #print("___", s1_)
        #print("S1 Cache", cache_collected, s1_collected, s1_total)

                #if complete:
    return s1_scores