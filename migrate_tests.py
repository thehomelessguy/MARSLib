import os
import re

TEST_DIR = "c:/Users/david/dev/robotics/frc/MARSLib/src/test/java/com/marslib"

def process_file(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    original_content = content

    # 1. Add import if needed
    if "import com.marslib.testing.MARSTestHarness;" not in content:
        # Find last import
        import_idx = content.rfind("import ")
        if import_idx != -1:
            end_of_line = content.find("\n", import_idx)
            content = content[:end_of_line+1] + "import com.marslib.testing.MARSTestHarness;\n" + content[end_of_line+1:]

    # 2. Lines to remove from ANYWHERE (mostly inside @BeforeEach / @AfterEach)
    lines_to_remove = [
        r"\s*HAL\.initialize\(500,\s*0\);?\n",
        r"\s*DriverStationSim\.setAllianceStationId\([\w\.]+\);?\n",
        r"\s*DriverStationSim\.setEnabled\(true\);?\n",
        r"\s*DriverStationSim\.notifyNewData\(\);?\n",
        r"\s*CommandScheduler\.getInstance\(\)\.cancelAll\(\);?\n",
        r"\s*CommandScheduler\.getInstance\(\)\.unregisterAllSubsystems\(\);?\n",
        r"\s*MARSPhysicsWorld\.resetInstance\(\);?\n",
        r"\s*MARSPhysicsWorld\.getInstance\(\)\.resetInstance\(\);?\n",
        r"\s*AprilTagVisionIOSim\.resetSimulation\(\);?\n"
    ]
    
    for pattern in lines_to_remove:
        content = re.sub(pattern, "", content)

    # 3. Insert MARSTestHarness.reset() to the top of @BeforeEach public void setUp()
    # It might be public void setUp() or public void setup() or void setUp()
    before_each_pattern = r"(@BeforeEach\s*\n\s*(?:public\s+)?void\s+set[Uu]p\(\)\s*\{)(?!\s*MARSTestHarness\.reset\(\);)"
    content = re.sub(before_each_pattern, r"\1\n    MARSTestHarness.reset();", content)

    # 4. Insert MARSTestHarness.tearDown() to the top of @AfterEach public void tearDown()
    after_each_pattern = r"(@AfterEach\s*\n\s*(?:public\s+)?void\s+tearDown\(\)\s*\{)(?!\s*MARSTestHarness\.tearDown\(\);)"
    content = re.sub(after_each_pattern, r"\1\n    MARSTestHarness.tearDown();", content)

    # 5. Remove empty @BeforeAll
    empty_before_all = r"\s*@BeforeAll\s*\n\s*(?:public\s+)?static\s+void\s+setup\(\)\s*\{\s*\}"
    content = re.sub(empty_before_all, "", content)

    if content != original_content:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        return True
    return False

modified_count = 0
for root, _, files in os.walk(TEST_DIR):
    for file in files:
        if file.endswith(".java"):
            filepath = os.path.join(root, file)
            if process_file(filepath):
                modified_count += 1
                print(f"Migrated: {file}")

print(f"Total migrated files: {modified_count}")
