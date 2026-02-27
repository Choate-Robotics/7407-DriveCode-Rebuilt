import json
import math
from pathlib import Path

FIELD_HEIGHT = 8

# flips a trench auto to the opposite trench
# change the "with open" line to the auto you want to flip
# does not work with pose waypoints that use degrees for now
# change choreo_dir to the name you want
# run in terminal 

    def waypoint_flip(output_path: Path): 
        with open("deploy/choreo/TwoLoopTopClimb.traj", "r") as f: 
            data = json.load(f)
            waypoints = data

        data["snapshot"]["waypoints"] = []
        data["trajectory"]["samples"] = [] # emptying out the path generated from the previous waypoints
        
        for waypoint in data["params"]["waypoints"]:
            waypoint["y"]["val"] = FIELD_HEIGHT - float(waypoint["y"]["val"]) # flips waypoint to other side of alliance side
            waypoint["y"]["exp"] = str(waypoint["y"]["val"]) + " m" # add "m" to signify meters for choreo
            waypoint["heading"]["val"] *= -1 #flips waypoint pose / rotation

            if "rad" in waypoint["heading"]["exp"]:
                waypoint["heading"]["exp"] = str(waypoint["heading"]["val"]) + " rad" # add "rad" to signify radians for choreo
            else:
                waypoint["heading"]["exp"] = str(waypoint["heading"]["val"] * (180/math.pi)) + " deg" # if your waypoints are in degrees

        data["trajectory"]["samples"] = []
            

        with open(output_path, "w") as f:
            json.dump(data, f, indent=2)

    if __name__ == "__main__":
        choreo_dir = Path("deploy/choreo")

        waypoint_flip(
            choreo_dir / "TwoLoopBottomClimb.traj"
        )