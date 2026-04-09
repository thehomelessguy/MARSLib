package frc.robot.subsystems;

import com.marslib.util.EliteShooterMath;
import com.marslib.util.MARSStateMachine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Superstructure orchestrator using a validated finite state machine. */
public class MARSSuperstructure extends SubsystemBase {

  private final MARSCowl cowl;
  private final MARSIntakePivot intakePivot;
  private final MARSShooter floorIntake;
  private final MARSShooter shooter;
  private final MARSShooter feeder;

  private final Supplier<Pose2d> poseSupplier;

  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Supplier<java.util.Optional<edu.wpi.first.math.geometry.Translation2d>>
      visionTargetSupplier;

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
      MARSCowl cowl,
      MARSIntakePivot intakePivot,
      MARSShooter floorIntake,
      MARSShooter shooter,
      MARSShooter feeder,
      Supplier<Pose2d> poseSupplier,
      Supplier<java.util.Optional<edu.wpi.first.math.geometry.Translation2d>>
          visionTargetSupplier) {

    this.cowl = cowl;
    this.intakePivot = intakePivot;
    this.floorIntake = floorIntake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.poseSupplier = poseSupplier;
    this.visionTargetSupplier = visionTargetSupplier;

    stateMachine =
        new MARSStateMachine<>(
            "Superstructure", SuperstructureState.class, SuperstructureState.STOWED);

    stateMachine.addValidBidirectional(SuperstructureState.STOWED, SuperstructureState.INTAKE_DOWN);
    stateMachine.addValidBidirectional(
        SuperstructureState.INTAKE_DOWN, SuperstructureState.INTAKE_RUNNING);
    stateMachine.addValidBidirectional(SuperstructureState.STOWED, SuperstructureState.SCORE);
    stateMachine.addValidBidirectional(SuperstructureState.STOWED, SuperstructureState.UNJAM);

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
        goalIntakeAngle = frc.robot.constants.SuperstructureConstants.INTAKE_PIVOT_ANGLE;
        goalCowlAngle = 0.0;
        break;
      case SCORE:
        goalIntakeAngle = 0.0;
        // Calculate static shot using EliteShooterMath with zero velocity
        edu.wpi.first.math.geometry.Translation2d hub =
            edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
                    && edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
                        == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                ? frc.robot.constants.FieldConstants.RED_HUB_POS
                : frc.robot.constants.FieldConstants.BLUE_HUB_POS;

        EliteShooterMath.EliteShooterSetpoint staticShot =
            EliteShooterMath.calculateShotOnTheMove(
                poseSupplier.get(),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                new edu.wpi.first.math.geometry.Translation3d(
                    hub.getX(), hub.getY(), frc.robot.constants.FieldConstants.HUB_SIZE_METERS),
                frc.robot.constants.FieldConstants.GAME_PIECE_REST_HEIGHT_METERS,
                frc.robot.constants.ShooterConstants.PROJECTILE_SPEED_MPS,
                -9.81,
                0.1);
        if (staticShot.isValid) {
          goalCowlAngle = staticShot.hoodRadians;
        } else {
          goalCowlAngle = frc.robot.constants.SuperstructureConstants.SCORE_FALLBACK_COWL_ANGLE;
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
                      frc.robot.constants.FieldConstants.INTAKE_COLLECTION_RADIUS_METERS,
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

    if (currentState == SuperstructureState.SCORE) {
      double targetRadPerSec = 4000.0 * Math.PI * 2.0 / 60.0; // Default fallback

      edu.wpi.first.math.geometry.Translation2d hubForRPM =
          edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
                  && edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
                      == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
              ? frc.robot.constants.FieldConstants.RED_HUB_POS
              : frc.robot.constants.FieldConstants.BLUE_HUB_POS;

      EliteShooterMath.EliteShooterSetpoint staticShotRpm =
          EliteShooterMath.calculateShotOnTheMove(
              poseSupplier.get(),
              new edu.wpi.first.math.kinematics.ChassisSpeeds(),
              new edu.wpi.first.math.geometry.Translation3d(
                  hubForRPM.getX(),
                  hubForRPM.getY(),
                  frc.robot.constants.FieldConstants.HUB_SIZE_METERS),
              frc.robot.constants.FieldConstants.GAME_PIECE_REST_HEIGHT_METERS,
              frc.robot.constants.ShooterConstants.PROJECTILE_SPEED_MPS,
              -9.81,
              0.1);

      if (staticShotRpm.isValid) {
        targetRadPerSec = staticShotRpm.launchSpeedMetersPerSec * 30.0;
      }

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
                      >= frc.robot.constants.FieldConstants.FIELD_LENGTH_METERS - allowedDistance);

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
                  frc.robot.constants.SuperstructureConstants.SHOOTER_EXIT_HEIGHT_METERS,
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
