import json
from pathlib import Path

FIELD_HEIGHT = 8

# flips a trench auto to the opposite trench
# change the "with open" line to the auto you want to flip
# does not work with pose waypoints that use degrees for now
# change choreo_dir to the name you want
# run in terminal with p

def waypoint_flip(output_path: Path): 
    with open("deploy/choreo/TwoLoopTopClimb.traj", "r") as f: 
        data = json.load(f)
        waypoints = data

    data["snapshot"]["waypoints"] = [] # i dont even know what snapshot does in a traj file tbh
    data["trajectory"]["samples"] = [] # emptying out the path generated from the previous waypoints
    
    for waypoint in data["params"]["waypoints"]:
        waypoint["y"]["val"] = FIELD_HEIGHT - float(waypoint["y"]["val"]) # flips waypoint to bottom of field
        waypoint["y"]["exp"] = str(waypoint["y"]["val"]) + " m" # flips waypoint to bottom of field
        
        waypoint["heading"]["val"] *= -1 #flips waypoint pose / rotation
        waypoint["heading"]["exp"] = str(waypoint["heading"]["val"]) + " rads" #flips waypoint pose / rotation

    data["trajectory"]["samples"] = []
        

    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

if __name__ == "__main__":
    choreo_dir = Path("deploy/choreo")

    waypoint_flip(
        choreo_dir / "TwoLoopBottomClimb.traj"
    )