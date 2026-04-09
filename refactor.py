import re

# 1. Update OperatorInterface.java
with open('src/main/java/com/marslib/hmi/OperatorInterface.java', 'r') as f:
    oi_content = f.read()

# Add imports to OperatorInterface
new_imports = """
import com.marslib.auto.GhostManager;
import com.marslib.auto.MARSDiagnosticCheck;
import com.marslib.auto.ShootOnTheMoveCommand;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.mechanisms.MARSShooter;
import com.marslib.mechanisms.MARSSuperstructure;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.TeleopDriveCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
"""
oi_content = oi_content.replace('import edu.wpi.first.wpilibj2.command.SubsystemBase;', new_imports + 'import edu.wpi.first.wpilibj2.command.SubsystemBase;')

# Add the configureBindings method
bindings_method = """
  public void configureBindings(
      SwerveDrive swerveDrive,
      GhostManager ghostManager,
      MARSSuperstructure superstructure,
      MARSElevator fastClimber,
      MARSArm cowl,
      MARSShooter feeder,
      MARSShooter floorIntake) {
      
    TelemetryGamepad coPilot = new TelemetryGamepad(1, "CoPilot");

    // 1. Default Commands
    swerveDrive.setDefaultCommand(
        new TeleopDriveCommand(
            swerveDrive,
            ghostManager,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));

    // 2. Drive Pilot Bindings
    controller.bindOnTrue(controller.leftTrigger(), "LeftTrigger", "Run Intake", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_RUNNING))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindWhileTrue(controller.rightTrigger(), "RightTrigger", "Aim & Shoot On Move", new ShootOnTheMoveCommand(
                swerveDrive,
                () -> {
                  double raw = -Math.pow(edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftY(), 0.1), 3.0) * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent() && edu.wpi.first.wpilibj.DriverStation.getAlliance().get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                },
                () -> {
                  double raw = -Math.pow(edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftX(), 0.1), 3.0) * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent() && edu.wpi.first.wpilibj.DriverStation.getAlliance().get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                }))
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindOnTrue(controller.b(), "B", "Stationary Shoot", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindOnTrue(controller.leftBumper(), "LeftBumper", "Unjam", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.UNJAM))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindOnTrue(controller.rightBumper(), "RightBumper", "Aim & Shuttle", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindOnTrue(controller.povRight(), "DPad_Right", "Deploy Intake Only", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_DOWN));

    controller.bindOnTrue(controller.povLeft(), "DPad_Left", "Retract Intake", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindOnTrue(controller.a(), "A", "Slamtake", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_RUNNING))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindWhileTrue(controller.povUp(), "DPad_Up", "Manual Climber Up", Commands.startEnd(
                () -> fastClimber.setVoltage(12.0),
                () -> fastClimber.setVoltage(0.0),
                fastClimber));

    controller.bindWhileTrue(controller.povDown(), "DPad_Down", "Manual Climber Down", Commands.startEnd(
                () -> fastClimber.setVoltage(-12.0),
                () -> fastClimber.setVoltage(0.0),
                fastClimber));

    controller.bindOnTrue(controller.back().and(controller.start()), "Back_And_Start", "Ghost Record", ghostManager.registerRecordCommand(
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> controller.getRightX(),
                controller.a(),
                controller.b(),
                controller.x(),
                controller.y(),
                controller.leftBumper(),
                controller.rightBumper(),
                controller.povUp(),
                controller.povDown(),
                controller.povLeft(),
                controller.povRight()));

    controller.bindOnTrue(controller.start(), "Start", "Diagnostic Check", new MARSDiagnosticCheck(swerveDrive, fastClimber, cowl));

    // 3. CoPilot Bindings
    coPilot.bindWhileTrue(coPilot.leftTrigger(), "LeftTrigger", "Manual Feed", Commands.startEnd(
                () -> {
                  feeder.setVoltage(6.0);
                  floorIntake.setVoltage(6.0);
                },
                () -> {
                  feeder.setVoltage(0.0);
                  floorIntake.setVoltage(0.0);
                },
                feeder,
                floorIntake));

    coPilot.bindOnTrue(coPilot.rightTrigger(), "RightTrigger", "Fixed Score (Hub)", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    coPilot.bindOnTrue(coPilot.rightBumper(), "RightBumper", "Fixed Score (Ladder)", superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    coPilot.bindOnTrue(coPilot.leftBumper(), "LeftBumper", "Cowl Home", cowl.home());

    coPilot.bindWhileTrue(coPilot.povDown(), "DPad_Down", "Climber Reverse", Commands.startEnd(
                () -> fastClimber.setVoltage(-12.0),
                () -> fastClimber.setVoltage(0.0),
                fastClimber));

    coPilot.bindOnTrue(coPilot.x(), "X", "Drivetrain Stop", Commands.runOnce(
                () -> swerveDrive.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds()),
                swerveDrive));
  }
}
"""

oi_content = oi_content.replace('}\n', bindings_method, 1)

with open('src/main/java/com/marslib/hmi/OperatorInterface.java', 'w') as f:
    f.write(oi_content)

# 2. Update RobotContainer.java
with open('src/main/java/frc/robot/RobotContainer.java', 'r') as f:
    rc_content = f.read()

# Replace method calls in constructor
rc_content = re.sub(
    r'// 2\. Configure Default Commands\s*configureDefaultCommands\(\);\s*// 3\. Configure Button Bindings\s*configureButtonBindings\(\);',
    'operatorInterface.configureBindings(swerveDrive, ghostManager, superstructure, fastClimber, cowl, feeder, floorIntake);',
    rc_content
)

# Remove the methods entirely
rc_content = re.sub(r'private void configureDefaultCommands\(\).*?public Command getAutonomousCommand\(\)', 'public Command getAutonomousCommand()', rc_content, flags=re.DOTALL)

with open('src/main/java/frc/robot/RobotContainer.java', 'w') as f:
    f.write(rc_content)
