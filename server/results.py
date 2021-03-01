
with open("./participants.txt", "r") as f:
  participants = f.readlines()
  for p in participants:
    p = p[:-1]

    print("looking at", p)

    with open("./saved_logs/" + p + ".txt", "r") as ff:
      actions = ff.readlines()

      started = False
      start_time = 0

      reset = False
      reset_time = 0
     
      complete = False
      end_time = 0
      for a in actions:
        # first get if sagat
        if "SAGAT" in a and "complete" in a:
          print("  SAGAT", a[:-1])

        if p + "," + p in a:
          output = a[a.find(p) + len(p) + 1:-1]
          print("  NETWORK", output)
          output = output.split(",")[1:-1]
          output = [int(x) for x in output]
          print("  OUTPUT", output, sum(output))
        
        if "stage" in a and a[-3] == "2" and "add-valid-waypoint" in a and not started:
          started = True
          start_time = int(a[:10])
          print("  start", a[:11])

        if "stage" in a and a[-3] == "2" and "add-valid-waypoint" in a and reset == True:
            reset = False
            reset_time = int(a[:10])
            print("    reset start", a[:11])

        if "'stage': 2, 'action': 'reset map'" in a:
          reset = True
          print("  reset!")

        if started == True and "'stage': 2, 'action': 'stage-complete'" in a:
          complete = True
          end_time = int(a[:10])
          print("  end", a[:11])
            

        if complete == True:
          print("  duration", end_time - start_time)
          if reset_time > 0:
            print("  reset duration", end_time - reset_time)
          break



        
