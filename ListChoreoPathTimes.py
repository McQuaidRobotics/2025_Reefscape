
CHOREO_TRAJ_DIR = "./src/deploy/choreo"


import os
import sys
import json

times = [("None", 0.0)]

for _, _, files in os.walk(CHOREO_TRAJ_DIR):
    for file in files:
        if file.endswith(".traj"):
            with open(f"{CHOREO_TRAJ_DIR}/{file}", "r") as f:
                jdata = json.load(f)
                try:
                    time = jdata["trajectory"]["samples"][-1]["t"]
                    name = jdata["name"]
                    times.append((name, time))
                except:
                    pass

for name, time in sorted(times, key=lambda x: x[1]):
    print(f"{name},{time}")