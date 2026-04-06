import glob
import re
import os

def find_taken_ids():
    pattern = re.compile(r'=\s*(\d+)\s*;')
    taken = []
    
    # Check Constants.java
    constants_path = "src/main/java/frc/robot/Constants.java"
    if os.path.exists(constants_path):
        with open(constants_path, 'r') as f:
            for line in f:
                if '_ID' in line and '=' in line:
                    m = pattern.search(line)
                    if m:
                        taken.append(("frc.robot.Constants", int(m.group(1))))
                        
    # Check all *IOTalonFX.java
    for filepath in glob.glob("src/main/java/com/marslib/**/*IOTalonFX.java", recursive=True):
        basename = os.path.basename(filepath)
        with open(filepath, 'r') as f:
            for line in f:
                # rudimentary check for new TalonFX(ID) or new CANSparkMax(ID
                m = re.search(r'new\s+(TalonFX|CANSparkMax)\s*\(\s*(\d+)', line)
                if m:
                    taken.append((basename, int(m.group(2))))
                    
    return taken

if __name__ == "__main__":
    taken = find_taken_ids()
    taken.sort(key=lambda x: x[1])
    
    print(f"| {'CAN ID':<6} | {'FILE / REFERENCE':<30} |")
    print("-" * 42)
    
    taken_nums = set()
    for ref, id_num in taken:
        print(f"| {id_num:<6} | {ref:<30} |")
        taken_nums.add(id_num)
        
    print("-" * 42)
    print("\n[RECOMMENDED NEXT IDs]")
    available = []
    for i in range(1, 63):
        if i not in taken_nums:
            available.append(i)
            if len(available) == 5:
                break
    print(f"Available Rio bus: {available}")
