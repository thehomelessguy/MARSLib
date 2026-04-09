import os
import re

print("Checking skills for missing classes...")
skills_dir = r"c:\Users\david\dev\robotics\frc\MARSLib\.agents\skills"
src_dir = r"c:\Users\david\dev\robotics\frc\MARSLib\src\main\java"

# Extract all java class names from skills
classes_in_skills = set()
skill_files = []

for root, dirs, files in os.walk(skills_dir):
    for f in files:
        if f == "SKILL.md":
            skill_files.append(os.path.join(root, f))

class_pattern = re.compile(r'\b([A-Z][a-zA-Z0-9_]*)(?:\.java)?\b')

for f_path in skill_files:
    with open(f_path, 'r', encoding='utf-8') as f:
        content = f.read()
        matches = class_pattern.findall(content)
        for match in matches:
            classes_in_skills.add((match, f_path))

# Extract all actual classes
actual_classes = set()
for root, dirs, files in os.walk(src_dir):
    for f in files:
        if f.endswith(".java"):
            actual_classes.add(f[:-5])

# Find non-existent classes
# Some might be third party like `Translation2d`, `Pose2d`, `SmartDashboard`, `Command`, `Subsystem`, etc.
# We'll just look for MARSLib specific ones or common FRC ones and identify suspicious ones.

suspicious_prefixes = ["MARS", "Ghost", "Array", "Linear", "Rotary", "Swerve", "Vision", "Auto", "Path", "Photon", "Ares"]

print("Potentially outdated class references in SKILL.md files:")
for cls, f_path in classes_in_skills:
    # Filter common words
    if cls in actual_classes:
        continue
    # If it starts with an interesting prefix
    is_interesting = any(cls.startswith(p) for p in suspicious_prefixes) or "IO" in cls or "Sim" in cls
    # Ignore some obvious third party
    if cls in ["SmartDashboard", "Command", "SubsystemBase", "Pose2d", "Transform3d", "Rotation2d", "Translation2d", "TalonFX", "CANSparkMax", "DcMotorEx"]:
        continue
    if is_interesting:
        print(f"{os.path.basename(os.path.dirname(f_path))}: {cls}")

