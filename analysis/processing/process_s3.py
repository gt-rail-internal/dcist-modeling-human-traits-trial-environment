# determines results for each user

import os
import math
#import networks_analysis

#path = "./logs_stage_3_sa/"

# euclidean distance formula
def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_s3_data(path, specific_users=[]):
    #print("PROCESSING S3 DATA")
    s3_scores = {}
    for p in os.listdir(path):
        # ignore replays temporarily
        if "replay" not in p or "S3" not in p:
            continue

        # check specific users
        if specific_users != [] and sum([1 for x in specific_users if x in p]) == 0:
            continue

        p = p[:-4]
        worker_id = p.split("-")[0]
        s3_scores[worker_id] = -1
        with open(path + "/" + p + ".txt", "r") as f:
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
            cache_locations = [[140, 208.6], [394.8, 228.9], [630, 261.8], [626.5, 613.9], [54.6, 530.6]]
            base_location = [406, 690.9]
            min_cache_dist = [294.7032405658275, 294.7032405658275, 329.2152862712491, 329.2152862712491, 581.7380457581583, 581.7380457581583, 255.74581870331477, 255.74581870331477, 314.70295872494506, 314.70295872494506]
            cache_states = [0, 0, 0, 0, 0]

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
                if "'stage': 3" in a and stage == 0:
                    stage = 3

                # start when a robot is no longer idle
                if stage == 3 and "no longer idle" in a and not started:
                    started = True
                    start_time = int(a[:10])

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
                
                # get the robot and cache states
                if stage == 3 and started == True and "locations" in a and not complete:
                    robot_locations = [float(x) for x in a.split("'")[13].split(":")[1].split(",")]
                    if len(a.split("'")) > 17 and len(a.split("'")[17].split(":")) > 1:
                        cache_states = [float(x) for x in a.split("'")[17].split(":")[1].split(",")]
                    # calculate the distance to each cache
                    for ii in range(len(cache_states)):  # for each cache
                        # parse the robot and cache state of each cache
                        cache_state = cache_states[ii] // 10  # the first digit is the cache state (0: init, 1: collected, 2: returned)
                        robot_tracking = cache_states[ii] % 10  # the second digit is the robot index that is carrying it
                        # for each robot
                        for r in range(len(robot_locations) // 3):
                            # get robot dist to the cache
                            cache_dist = distance(cache_locations[ii], [robot_locations[3 * r], robot_locations[3 * r + 1]])
                            # get robot dist to the base
                            base_dist = distance([robot_locations[3 * r], robot_locations[3 * r + 1]], base_location)
                            # if the cache state is init, update if robot distance is less than current recorded distance
                            if cache_state == 0:
                                min_cache_dist[ii * 2] = cache_dist if cache_dist < min_cache_dist[ii * 2] else min_cache_dist[ii * 2]  # ii * 2 because the encoding is [cache1, cache1base, cache2, cache2base, etc]
                            # if the cache state is collected, update if the robot carrying it has a lower distance to the base than what is recorded
                            if cache_state == 1:
                                min_cache_dist[ii * 2] = 0
                                min_cache_dist[ii * 2 + 1] = base_dist if base_dist < min_cache_dist[ii * 2] else min_cache_dist[ii * 2]
                            # if the cache state is returned, set both distances to 0
                            if cache_state == 2:
                                min_cache_dist[ii * 2] = 0
                                min_cache_dist[ii * 2 + 1] = 0

                if stage == 3 and started == True and ("stage-complete" in a or int(a[:10]) - start_time > 60 * 10 or i == len(actions)-1):
                    #print("complete")
                    complete = True
                    end_time = int(a[:10])

                if stage == 3 and complete == True:
                    duration = 600 #end_time - start_time  # only one user completed the stage, so can consider all users at 600
                    # the initial cache locations
                    initial_dist = 2 * sum([294.7032405658275, 329.2152862712491, 581.7380457581583, 255.74581870331477, 314.70295872494506])
                    s3_scores[worker_id] = (initial_dist - sum(min_cache_dist)) / duration

                    break
                
                former_a = a

               
    normalization = max(s3_scores.values())  # normalization factor
    s3_scores = {p : s3_scores[p] / normalization for p in s3_scores}

    return s3_scores
