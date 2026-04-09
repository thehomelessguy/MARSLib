package frc.robot;

import com.marslib.auto.GhostManager;
import com.marslib.hmi.TelemetryGamepad;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.TeleopDriveCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.MARSDiagnosticCheck;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.MARSClimber;
import frc.robot.subsystems.MARSCowl;
import frc.robot.subsystems.MARSShooter;
import frc.robot.subsystems.MARSSuperstructure;
import frc.robot.subsystems.OperatorInterface;

public final class RobotBindings {

  private RobotBindings() {}

  public static void configureBindings(
      OperatorInterface operatorInterface,
      SwerveDrive swerveDrive,
      GhostManager ghostManager,
      MARSSuperstructure superstructure,
      MARSClimber climber,
      MARSCowl cowl,
      MARSShooter shooter,
      MARSShooter feeder,
      MARSShooter floorIntake) {

    TelemetryGamepad controller = operatorInterface.getController();
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
    controller
        .bindOnTrue(
            controller.leftTrigger(),
            "LeftTrigger",
            "Run Intake",
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_RUNNING))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller
        .bindWhileTrue(
            controller.rightTrigger(),
            "RightTrigger",
            "Aim And Shoot On Move",
            new ShootOnTheMoveCommand(
                swerveDrive,
                cowl,
                shooter,
                () -> {
                  double raw =
                      -Math.pow(
                              edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftY(), 0.1),
                              3.0)
                          * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
                      && edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
                          == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                },
                () -> {
                  double raw =
                      -Math.pow(
                              edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftX(), 0.1),
                              3.0)
                          * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
                      && edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
                          == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                }))
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller
        .bindOnTrue(
            controller.b(),
            "B",
            "Stationary Shoot",
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller
        .bindOnTrue(
            controller.leftBumper(),
            "LeftBumper",
            "Unjam",
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.UNJAM))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller
        .bindOnTrue(
            controller.rightBumper(),
            "RightBumper",
            "Aim And Shuttle",
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindOnTrue(
        controller.povRight(),
        "DPad_Right",
        "Deploy Intake Only",
        superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_DOWN));

    controller.bindOnTrue(
        controller.povLeft(),
        "DPad_Left",
        "Retract Intake",
        superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller
        .bindOnTrue(
            controller.a(),
            "A",
            "Slamtake",
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_RUNNING))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    controller.bindWhileTrue(
        controller.y(),
        "Y",
        "Align To Climb Position",
        swerveDrive.alignToPoint(
            () ->
                frc.robot.constants.FieldConstants.getClosestClimbingPosition(
                    swerveDrive.getPose())));

    controller.bindOnTrue(
        controller.x(), "X", "Final Climb Lineup", swerveDrive.finalClimbLineupCommand());

    controller.bindWhileTrue(
        controller.povUp(),
        "DPad_Up",
        "Manual Climber Up",
        Commands.startEnd(() -> climber.setVoltage(12.0), () -> climber.setVoltage(0.0), climber));

    controller.bindWhileTrue(
        controller.povDown(),
        "DPad_Down",
        "Manual Climber Down",
        Commands.startEnd(() -> climber.setVoltage(-12.0), () -> climber.setVoltage(0.0), climber));

    controller.bindOnTrue(
        controller.back().and(controller.start()),
        "Back_And_Start",
        "Ghost Record",
        ghostManager.registerRecordCommand(
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

    controller.bindOnTrue(
        controller.start(),
        "Start",
        "Diagnostic Check",
        new MARSDiagnosticCheck(swerveDrive, climber, cowl));

    // 3. CoPilot Bindings
    coPilot.bindWhileTrue(
        coPilot.leftTrigger(),
        "LeftTrigger",
        "Manual Feed",
        Commands.startEnd(
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

    coPilot
        .bindOnTrue(
            coPilot.rightTrigger(),
            "RightTrigger",
            "Fixed Score (Hub)",
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    coPilot
        .bindOnTrue(
            coPilot.rightBumper(),
            "RightBumper",
            "Fixed Score (Ladder)",
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    coPilot.bindOnTrue(coPilot.leftBumper(), "LeftBumper", "Cowl Home", cowl.home());

    coPilot.bindWhileTrue(
        coPilot.povDown(),
        "DPad_Down",
        "Climber Reverse",
        Commands.startEnd(() -> climber.setVoltage(-12.0), () -> climber.setVoltage(0.0), climber));

    coPilot.bindOnTrue(
        coPilot.x(),
        "X",
        "Drivetrain Stop",
        Commands.runOnce(
            () -> swerveDrive.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds()),
            swerveDrive));
  }
}
