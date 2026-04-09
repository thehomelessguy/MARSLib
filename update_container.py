import re

with open('src/main/java/frc/robot/RobotContainer.java', 'r') as f:
    content = f.read()

# Add import
content = content.replace(
    'import edu.wpi.first.wpilibj2.command.button.CommandXboxController;',
    'import edu.wpi.first.wpilibj2.command.button.CommandXboxController;\nimport com.marslib.hmi.TelemetryGamepad;'
)

# Replace local variables
content = content.replace(
    'CommandXboxController controller = operatorInterface.getController();',
    'TelemetryGamepad controller = operatorInterface.getController();'
)

content = content.replace(
    'CommandXboxController coPilot = new CommandXboxController(1);',
    'TelemetryGamepad coPilot = new TelemetryGamepad(1, "CoPilot");'
)

# Define regex patterns for replacements
replacements = [
    (r'controller\s*\n*\s*\.leftTrigger\(\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.leftTrigger(), "LeftTrigger", "Run Intake", '),
     
    (r'controller\s*\n*\s*\.rightTrigger\(\)\s*\n*\s*\.whileTrue\(', 
     r'controller.bindWhileTrue(controller.rightTrigger(), "RightTrigger", "Aim & Shoot On Move", '),
     
    (r'controller\s*\n*\s*\.b\(\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.b(), "B", "Stationary Shoot", '),
     
    (r'controller\s*\n*\s*\.leftBumper\(\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.leftBumper(), "LeftBumper", "Unjam", '),
     
    (r'controller\s*\n*\s*\.rightBumper\(\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.rightBumper(), "RightBumper", "Aim & Shuttle", '),
     
    (r'controller\s*\n*\s*\.povRight\(\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.povRight(), "DPad_Right", "Deploy Intake Only", '),
     
    (r'controller\s*\n*\s*\.povLeft\(\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.povLeft(), "DPad_Left", "Retract Intake", '),
     
    (r'controller\s*\n*\s*\.a\(\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.a(), "A", "Slamtake", '),
     
    (r'controller\s*\n*\s*\.povUp\(\)\s*\n*\s*\.whileTrue\(', 
     r'controller.bindWhileTrue(controller.povUp(), "DPad_Up", "Manual Climber Up", '),
     
    (r'controller\s*\n*\s*\.povDown\(\)\s*\n*\s*\.whileTrue\(', 
     r'controller.bindWhileTrue(controller.povDown(), "DPad_Down", "Manual Climber Down", '),     
     
    (r'controller\s*\n*\s*\.back\(\)\s*\n*\s*\.and\(controller\.start\(\)\)\s*\n*\s*\.onTrue\(', 
     r'controller.bindOnTrue(controller.back().and(controller.start()), "Back_And_Start", "Ghost Record", '),

    (r'controller\.start\(\)\.onTrue\(', 
     r'controller.bindOnTrue(controller.start(), "Start", "Diagnostic Check", '),
     
    (r'coPilot\s*\n*\s*\.leftTrigger\(\)\s*\n*\s*\.whileTrue\(', 
     r'coPilot.bindWhileTrue(coPilot.leftTrigger(), "LeftTrigger", "Manual Feed", '),
     
    (r'coPilot\s*\n*\s*\.rightTrigger\(\)\s*\n*\s*\.onTrue\(', 
     r'coPilot.bindOnTrue(coPilot.rightTrigger(), "RightTrigger", "Fixed Score (Hub)", '),
     
    (r'coPilot\s*\n*\s*\.rightBumper\(\)\s*\n*\s*\.onTrue\(', 
     r'coPilot.bindOnTrue(coPilot.rightBumper(), "RightBumper", "Fixed Score (Ladder)", '),
     
    (r'coPilot\.leftBumper\(\)\.onTrue\(', 
     r'coPilot.bindOnTrue(coPilot.leftBumper(), "LeftBumper", "Cowl Home", '),
     
    (r'coPilot\s*\n*\s*\.povDown\(\)\s*\n*\s*\.whileTrue\(', 
     r'coPilot.bindWhileTrue(coPilot.povDown(), "DPad_Down", "Climber Reverse", '),
     
    (r'coPilot\s*\n*\s*\.x\(\)\s*\n*\s*\.onTrue\(', 
     r'coPilot.bindOnTrue(coPilot.x(), "X", "Drivetrain Stop", ')
]

for pattern, replacement in replacements:
    content = re.sub(pattern, replacement, content)

with open('src/main/java/frc/robot/RobotContainer.java', 'w') as f:
    f.write(content)
