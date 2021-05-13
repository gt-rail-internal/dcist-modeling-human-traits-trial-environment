# replays Stage 3 from the log file of user actions

import requests
import ast
import datetime
import time


def add_waypoint(robot, x, y):
    requests.get("http://fbd7eaf459e6.ngrok.io/add-waypoint?id=" + robot + "&x=" + str(x) + "&y=" + str(y))
    return

def remove_waypoint(robot):
    requests.get("http://fbd7eaf459e6.ngrok.io/remove-waypoint?id=" + robot)
    return

def replay_stage(worker_id):
    start_time = float("inf")
    diff_time = 0
    running = False

    with open("./logs/" + worker_id + ".txt", "r") as f:
        print("found log")
        lines = f.readlines()
        lines = [x[:-1] for x in lines]  # remove the trailing \n

        for action in lines:
            # skip blank lines
            if action == "":
                continue

            # parse the action
            comma = action.find(",")
            this_time = float(action[:comma])
            curr_time = datetime.datetime.now().timestamp() - diff_time

            # first get the first action
            if not running and "add-valid-waypoint" in action and "'stage': 1" in action:
                start_time = float(action.split(",")[0])
                diff_time = datetime.datetime.now().timestamp() - start_time
                running = True
                print("starting run", action)
            
            # then check if at least the start time
            if this_time < start_time:
                continue
                    
            # then check if adding or removing a waypoint
            if running and "add-valid-waypoint" not in action and "remove-waypoint" not in action:
                print("    ", action)
                continue

            # at this point action is either add valid waypoint or remove, so wait until time to execute
            while curr_time < this_time:
                time.sleep(.1)
                curr_time = datetime.datetime.now().timestamp() - diff_time
            
            if "add-valid-waypoint" in action:
                entry = ast.literal_eval(action[comma + len(worker_id) + 2:])
                location = ast.literal_eval(entry["location"])
                add_waypoint(entry["target"], location[0], location[1])
                print("added waypoint", action)

            if "remove-waypoint" in action:
                entry = ast.literal_eval(action[comma + len(worker_id) + 2:])
                remove_waypoint(entry["target"])
                print("removed waypoint")
    print("done")

replay_stage("A1Q7FDGYMNEXTE")
