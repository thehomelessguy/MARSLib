package com.marslib.mechanisms;

import com.marslib.util.MARSStateMachine;
import com.marslib.util.ShotSetup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Superstructure orchestrator using a validated finite state machine. */
public class MARSSuperstructure extends SubsystemBase {

  private final MARSArm cowl;
  private final MARSArm intakePivot;
  private final MARSShooter floorIntake;
  private final MARSShooter shooter;
  private final MARSShooter feeder;

  private final Supplier<Pose2d> poseSupplier;
  private final DoubleSupplier distanceSupplier;
  private final ShotSetup shotSetup;

  public enum SuperstructureState {
    STOWED,
    INTAKE_DOWN,
    INTAKE_RUNNING,
    SCORE,
    UNJAM
  }

  private final MARSStateMachine<SuperstructureState> stateMachine;
  private int gamePieceCount = 0;

  private double goalCowlAngle = 0.0;
  private double goalIntakeAngle = 0.0;

  public MARSSuperstructure(
      MARSArm cowl,
      MARSArm intakePivot,
      MARSShooter floorIntake,
      MARSShooter shooter,
      MARSShooter feeder,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier distanceSupplier,
      ShotSetup shotSetup) {

    this.cowl = cowl;
    this.intakePivot = intakePivot;
    this.floorIntake = floorIntake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.poseSupplier = poseSupplier;
    this.distanceSupplier = distanceSupplier;
    this.shotSetup = shotSetup;

    stateMachine =
        new MARSStateMachine<>(
            "Superstructure", SuperstructureState.class, SuperstructureState.STOWED);

    // Allowing wildcard transitions since there are no harsh collision paths on the decoupled MXIX
    // chassis
    for (SuperstructureState state : SuperstructureState.values()) {
      stateMachine.addWildcardFrom(state);
    }

    stateMachine.setEntryAction(
        SuperstructureState.SCORE,
        () -> {
          Logger.recordOutput("Superstructure/EntryAction", "SCORE: Readying shot");
        });

    stateMachine.setOnTransition(
        (from, to) ->
            Logger.recordOutput(
                "Superstructure/TransitionDetail",
                String.format(
                    "%s→%s at cowl=%.3frad intake=%.3frad gamePieceCount=%d",
                    from.name(),
                    to.name(),
                    cowl.getPositionRads(),
                    intakePivot.getPositionRads(),
                    gamePieceCount)));
  }

  public Command setAbsoluteState(SuperstructureState targetState) {
    return Commands.runOnce(
        () -> {
          boolean accepted = stateMachine.requestTransition(targetState);
          if (!accepted) {
            Logger.recordOutput(
                "Superstructure/TransitionRejectedReason",
                String.format(
                    "%s→%s is not a legal transition.",
                    stateMachine.getState().name(), targetState.name()));
          }
        },
        this);
  }

  public void forceState(SuperstructureState targetState) {
    stateMachine.requestTransition(SuperstructureState.STOWED);
    stateMachine.requestTransition(targetState);
  }

  @Override
  public void periodic() {
    stateMachine.update();
    SuperstructureState currentState = stateMachine.getState();

    switch (currentState) {
      case INTAKE_DOWN:
      case INTAKE_RUNNING:
        goalIntakeAngle = frc.robot.Constants.SuperstructureConstants.INTAKE_ARM_ANGLE;
        goalCowlAngle = 0.0;
        break;
      case SCORE:
        goalIntakeAngle = 0.0;
        // Dynamically compute cowl angle based on distance
        if (shotSetup != null) {
          goalCowlAngle = shotSetup.getStaticShotInfo(distanceSupplier.getAsDouble()).cowlPosition;
        } else {
          goalCowlAngle = frc.robot.Constants.SuperstructureConstants.SCORE_HIGH_ARM_ANGLE;
        }
        break;
      case STOWED:
      case UNJAM:
      default:
        goalIntakeAngle = 0.0;
        goalCowlAngle = 0.0;
        break;
    }

    cowl.setTargetPosition(goalCowlAngle);
    intakePivot.setTargetPosition(goalIntakeAngle);

    // Collision check from physics engine
    if (currentState == SuperstructureState.INTAKE_RUNNING && gamePieceCount < 40) {
      if (intakePivot.isAtTolerance()) {
        try {
          int swallowed =
              com.marslib.simulation.MARSPhysicsWorld.getInstance()
                  .checkIntake(
                      poseSupplier.get(),
                      frc.robot.Constants.FieldConstants.INTAKE_COLLECTION_RADIUS_METERS,
                      40 - gamePieceCount);
          gamePieceCount += swallowed;
        } catch (Exception e) {
          Logger.recordOutput(
              "Superstructure/IntakeError", e.getClass().getSimpleName() + ": " + e.getMessage());
        }
      }
    }

    // Run intake motors
    if (currentState == SuperstructureState.INTAKE_RUNNING && gamePieceCount < 40) {
      floorIntake.setVoltage(12.0);
    } else if (currentState != SuperstructureState.SCORE
        && currentState != SuperstructureState.UNJAM) {
      floorIntake.setVoltage(0.0);
    }

    // Run scoring / transfer sequence
    if (currentState == SuperstructureState.SCORE) {
      double targetRPM = 4000.0; // Default fallback
      if (distanceSupplier != null && shotSetup != null) {
        targetRPM = shotSetup.getStaticShotInfo(distanceSupplier.getAsDouble()).shot.shooterRPM;
      }

      double targetRadPerSec = targetRPM * Math.PI * 2.0 / 60.0;
      shooter.setClosedLoopVelocity(targetRadPerSec);

      // Wait for flywheel and cowl to be at tolerance before transferring
      if (shooter.isAtTolerance() && cowl.isAtTolerance()) {
        feeder.setVoltage(12.0);
        floorIntake.setVoltage(12.0);

        if (gamePieceCount > 0) {
          Pose2d robotPose = poseSupplier.get();
          var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
          boolean isBlue =
              alliance.isPresent()
                  && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

          // Ensure on the correct side (182.11 inches or 4.625594 m from end of field)
          double allowedDistance = 4.625594;
          boolean correctSide =
              isBlue
                  ? (robotPose.getX() <= allowedDistance)
                  : (robotPose.getX()
                      >= frc.robot.Constants.FieldConstants.FIELD_LENGTH_METERS - allowedDistance);

          // Shoot the piece in whatever direction the robot is facing
          double vx = 15.0 * Math.cos(robotPose.getRotation().getRadians());
          double vy = 15.0 * Math.sin(robotPose.getRotation().getRadians());
          double vz = 5.0;

          com.marslib.simulation.MARSPhysicsWorld.getInstance()
              .launchGamePiece(
                  robotPose.getTranslation(),
                  vx,
                  vy,
                  vz,
                  frc.robot.Constants.SuperstructureConstants.SCORE_HIGH_ELEVATOR_HEIGHT,
                  correctSide);
          gamePieceCount--;
        }
      } else {
        feeder.setVoltage(0.0);
        floorIntake.setVoltage(0.0);
      }
    } else if (currentState != SuperstructureState.UNJAM) {
      double currentRPM = shooter.getVelocityRadPerSec() * 60.0 / (Math.PI * 2.0);
      if (currentRPM > 1600.0) {
        shooter.setVoltage(0.0); // Coast down via friction
      } else {
        double idleRadPerSec = 1500.0 * Math.PI * 2.0 / 60.0;
        shooter.setClosedLoopVelocity(idleRadPerSec); // Maintain idle speed
      }
      feeder.setVoltage(0.0);
    }

    // Unjam mechanism
    if (currentState == SuperstructureState.UNJAM) {
      feeder.setVoltage(-6.0);
      floorIntake.setVoltage(-6.0);
      shooter.setVoltage(-6.0);
    }

    Logger.recordOutput("Superstructure/GamePieceCount", gamePieceCount);
    Logger.recordOutput("Superstructure/GoalCowlAngle", goalCowlAngle);
    Logger.recordOutput("Superstructure/GoalIntakeAngle", goalIntakeAngle);
    Logger.recordOutput("Superstructure/CurrentState", currentState.name());
  }

  public SuperstructureState getCurrentState() {
    return stateMachine.getState();
  }

  public MARSStateMachine<SuperstructureState> getStateMachine() {
    return stateMachine;
  }
}
