import json
from pathlib import Path

FIELD_HEIGHT = 8

def waypoint_flip(output_path: Path):
    with open("deploy/choreo/TwoLoopTopClimb.traj", "r") as f:
        data = json.load(f)
        waypoints = data

    
    for waypoint in data["waypoints"]:
        waypoint["y"] = FIELD_HEIGHT - waypoint["y"] # flips waypoint to bottom of field
        
        waypoint["heading"] *= -1 #flips waypoint pose / rotation

    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

if __name__ == "__main__":
    choreo_dir = Path("src/main/deploy/choreo")

    waypoint_flip(
        choreo_dir / "TwoLoopBottomClimb.traj"
    )