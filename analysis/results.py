# determines results for each user

import os
import networks_analysis

for p in os.listdir("./logs_stage_3"):
  p = p[:-4]
  with open("./logs_stage_3/" + p + ".txt", "r") as f:
    print("----", p)
    response = "Looking at user" + " " + p

    actions = f.readlines()

    started = False
    start_time = 0

    reset = False
    reset_time = 0
    
    complete = False
    end_time = 0

    robots_interacted = [0, 0, 0, 0]
    num_interactions = [0, 0, 0, 0]
    distance_traveled = []

    cache_collected = 0
    cache_returned = 0
    cache_identified = 0

    stage = 0

    former_a = ""

    for a in actions:
      # if completed the SAGAT, print the score
      if "SAGAT" in a and "complete" in a:
        response += "\n" + "  SAGAT " + str(a[:-1])

      # if completed the network connectivity pretest, print the score
      if p + "," + p in a:
        output = a[a.find(p) + len(p) + 1:-1]
        output = output.split(",")[1:-1]
        output = [int(x) for x in output]
        score = networks_analysis.determine_score(output)
        if score != -1:
          response += "\n" + "  NETWORK " + str(score)
        else:
          response += "\n" + "  INVALID NETWORK"

      # if completed Stage 1, get stats
      if "stage" in a and a[-3] == "1" and stage == 0:
        stage = 1

      if stage == 1 and "add-valid-waypoint" in a and not started:
        started = True
        start_time = int(a[:10])
        response += "\n" + "  Stage 1 Start " + a[:11]

      if stage == 1 and "cache collected" in a:
        cache_collected += 1
      
      if stage == 1 and started == True and "stage-complete" in a:
        complete = True
        end_time = int(a[:10])
        response += "\n" + "  Number of caches identified: " + str(cache_collected)
        response += "\n" + "  Stage 1 End " + a[:11]

      if stage == 1 and complete == True:
        response += "\n" + "  Stage 1 duration " + str(end_time - start_time)
        break

      # if completed Stage 2, print the start time, reset times, end time, and the duration
      if "stage" in a and a[-3] == "2" and stage == 0:
        stage = 2
      
      if stage == 2 and "add-valid-waypoint" in a and not started:
        started = True
        start_time = int(a[:10])
        response += "\n" + "  Stage 2 Start " + a[:11]

      if stage == 2 and "add-valid-waypoint" in a and reset == True:
          reset = False
          reset_time = int(a[:10])
          response += "\n" + "    Stage 2 reset start " + a[:11]

      if "'stage': 2, 'action': 'reset map'" in a:
        reset = True
        response += "\n" + "  Stage 2 reset!"

      if stage == 2 and started == True and "'stage': 2, 'action': 'stage-complete'" in a:
        complete = True
        end_time = int(a[:10])
        response += "\n" + "  Stage 2 End " + a[:11]

      if stage == 2 and complete == True:
        response += "\n" + "  Stage 2 duration" + str(end_time - start_time)
        if reset_time > 0:
          response += "\n" + "  Stage 2 reset duration" + str(end_time - reset_time)
        break

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

      former_a = a
    
    #if complete:
    print(response)



        
