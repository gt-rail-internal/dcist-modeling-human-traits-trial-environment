# replays Stage 3 from the log file of user actions

import requests
import ast
import datetime
import time
from selenium import webdriver

DOMAIN = "35b4-143-215-178-206"
STAGE = 2
USER = "7310"
BROWSER = None

# open the stage in a new browser instance
def open_stage():
    #global DOMAIN, STAGE, USER
    global BROWSER
    url = "http://" + DOMAIN + ".ngrok.io/stage?stage=" + str(STAGE) + "&workerId=" + str(USER) + "-replay-S" + str(STAGE)
    BROWSER = webdriver.Firefox()
    BROWSER.get(url)

# clicks all the collect cache buttons
def collect_cache():
    cam1 = BROWSER.find_element_by_id("cam1_button")
    cam2 = BROWSER.find_element_by_id("cam2_button")
    cam3 = BROWSER.find_element_by_id("cam3_button")
    cam4 = BROWSER.find_element_by_id("cam4_button")

    cam1.click()
    cam2.click()
    cam3.click()
    cam4.click()
    return

def add_waypoint(robot, x, y):
    requests.get("http://" + DOMAIN + ".ngrok.io/add-waypoint?id=" + robot + "&x=" + str(x) + "&y=" + str(y))
    return

def remove_waypoint(robot):
    requests.get("http://" + DOMAIN + ".ngrok.io/remove-waypoint?id=" + robot)
    return

def replay_stage(worker_id):
    start_time = float("inf")
    diff_time = 0
    running = False
    open_stage()  # the browser that we are emulating
    time.sleep(10)  # wait 10 secs

    with open("./logs/" + str(worker_id) + ".txt", "r") as f:
        print("found log")
        lines = f.readlines()
        lines = [x[:-1] for x in lines]  # remove the trailing \n

        for action in lines:
            # skip blank lines
            if action == "":
                continue

            # ignore if not the correct stage
            if "'stage': " + str(STAGE) not in action:
                continue

            # parse the action
            comma = action.find(",")
            this_time = float(action[:comma])
            curr_time = datetime.datetime.now().timestamp() - diff_time

            # first get the first action
            if not running and ("add-valid-waypoint" in action or "select-vehicle" in action) and "'stage': " + str(STAGE) in action:
                start_time = float(action.split(",")[0])
                diff_time = datetime.datetime.now().timestamp() - start_time
                running = True
                BROWSER.execute_script("startStage();")
                print("starting run", action)
            
            # then check if at least the start time
            if this_time < start_time:
                continue
                    
            # then check if adding or removing a waypoint
            if running and "select-vehicle" not in action and "deselect-vehicle" not in action and "add-valid-waypoint" not in action and "remove-waypoint" not in action and "cache" not in action.lower():
                print("    ", action)
                continue

            # at this point action is either add valid waypoint or remove, so wait until time to execute
            while curr_time < this_time:
                time.sleep(.1)
                curr_time = datetime.datetime.now().timestamp() - diff_time

            # select a robot
            if "select-vehicle" in action:
                target_robot = action.split("'")[7]
                if STAGE == 2:
                    target_robot_dict = {"UGV1": 5, "UGV2": 6, "UGV3": 7, "UGV4": 8, "UAV1": 1, "UAV2": 2, "UAV3": 3, "UAV4": 4}
                target_robot_index = target_robot_dict[target_robot]
                BROWSER.execute_script("uiMap.selectedObject = uiMap.uiObjects[" + str(target_robot_index) + "];")
                print("selected vehicle")

            if "deselect-vehicle" in action:
                act = webdriver.common.action_chains.ActionChains(BROWSER)
                act.send_keys("q")
                act.perform()
                print("removed waypoint")
            
            if "add-valid-waypoint" in action:
                entry = ast.literal_eval(action[comma + len(worker_id) + 2:])
                location = ast.literal_eval(entry["location"])
                add_waypoint(entry["target"], location[0], location[1])
                stage = BROWSER.find_element_by_id("uimap-canvas")
                act = webdriver.common.action_chains.ActionChains(BROWSER)
                act.move_to_element_with_offset(stage, 700 * location[0], 700 * location[1])
                act.click().perform()

                print("added waypoint", action)

            if "remove-waypoint" in action:
                act = webdriver.common.action_chains.ActionChains(BROWSER)
                act.send_keys("r")
                act.perform()
                print("removed waypoint")
            
            if "cache" in action.lower() and "cache connected" not in action.lower():
                collect_cache()
                print("collected cache")

    print("done")

replay_stage(USER)
