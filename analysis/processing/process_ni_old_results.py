# determines results for each user

import os
import process_ni_old

puzzle_scores = [[], [], [], [], []]
puzzle_raws = [[], [], [], [], []]

for p in os.listdir("./logs_stage_1"):
  p = p[:-4]
  with open("./logs_stage_1/" + p + ".txt", "r") as f:
    actions = f.readlines()

    started = False
    start_time = 0

    reset = False
    reset_time = 0
    
    complete = False
    end_time = 0

    robots_interacted = [0, 0, 0, 0]
    num_interactions = [0, 0, 0, 0]

    cache_collected = 0
    cache_returned = 0
    cache_identified = 0

    stage = 0

    former_a = ""

    for a in actions:
      # if completed the network connectivity pretest, print the score
      if p + "," + p in a:
        print("----", p)
        response = ""
        output = a[a.find(p) + len(p) + 1:-1]
        output = output.split(",")[1:-1]
        output = [int(x) for x in output]
        total, scores, raws = networks_analysis.determine_score(output)
        if total != -1:
          response += "  Total " + str(total)
          response += "  Scores " + str(scores)
          puzzle_scores[0].append(scores[0])
          puzzle_scores[1].append(scores[1])
          puzzle_scores[2].append(scores[2])
          puzzle_scores[3].append(scores[3])
          puzzle_scores[4].append(scores[4])

          # add the raws to the puzzle raws
          for raw in raws[0]:
            puzzle_raws[0].append(raw)
          for raw in raws[1]:
            puzzle_raws[1].append(raw)
          for raw in raws[2]:
            puzzle_raws[2].append(raw)
          for raw in raws[3]:
            puzzle_raws[3].append(raw)
          for raw in raws[4]:
            puzzle_raws[4].append(raw)
          
        else:
          response += "  INVALID NETWORK"

        #if complete:
        print(response)

# print the summary results for each puzzle
print("Average Score per Puzzle per Person")
puzzle_count = 1
for puzzle in puzzle_scores:
  print("\nAverage Puzzle", puzzle_count, "len", len(puzzle))
  puzzle_count += 1
  for score in puzzle:
    print(score)

print("Raw Score per Puzzle per Person")
puzzle_count = 1
for puzzle in puzzle_raws:
  print("\nRaw Puzzle", puzzle_count, "len", len(puzzle))
  puzzle_count += 1
  for score in puzzle:
    print(score)
