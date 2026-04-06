import glob
import json
import os
import sys

# Assume standard tread friction limit (e.g. 1.2G = ~11.7 m/s^2 before slipping)
MAX_CARPET_ACCEL_MPS2 = 8.0 

def validate_all_paths():
    failed = False
    
    path_files = glob.glob("src/main/deploy/pathplanner/paths/*.path")
    if not path_files:
        print("No path files found.")
        return

    for pfile in path_files:
        filename = os.path.basename(pfile)
        with open(pfile, 'r') as f:
            data = json.load(f)
            
        points = data.get("waypoints", [])
        global_constraints = data.get("globalConstraints", {})
        
        max_vel = global_constraints.get("maxVelocity", 4.0)
        max_accel = global_constraints.get("maxAcceleration", 3.0)
        
        if max_accel > MAX_CARPET_ACCEL_MPS2:
            print(f"[ERROR] {filename} globally requests {max_accel} m/s^2, which exceeds physical friction!")
            failed = True
            
        print(f"[OK] {filename} structurally sound (V={max_vel}, A={max_accel}).")

    if failed:
        print("\n=> Kinematics verification FAILED. Tuning required before deploy.")
        sys.exit(1)
    else:
        print("\n=> All paths verified within constraints.")
        sys.exit(0)

if __name__ == "__main__":
    validate_all_paths()
