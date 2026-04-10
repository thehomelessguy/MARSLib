package frc.robot.subsystems;

import com.marslib.util.AllianceUtil;
import com.marslib.util.EliteShooterMath;
import com.marslib.util.MARSStateMachine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SuperstructureConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure orchestrator using a validated finite state machine.
 *
 * <p>Coordinates the cowl, intake pivot, floor intake, shooter, and feeder subsystems through safe
 * state transitions. Automatically enters {@link SuperstructureState#BEACHED BEACHED} mode when the
 * robot tilt exceeds 25°, disabling all mechanisms to prevent damage during field obstacle
 * traversal.
 */
public class MARSSuperstructure extends SubsystemBase {

  private final MARSCowl cowl;
  private final MARSIntakePivot intakePivot;
  private final MARSShooter floorIntake;
  private final MARSShooter shooter;
  private final MARSShooter feeder;

  private final Supplier<Pose2d> poseSupplier;

  @SuppressWarnings({"PMD.UnusedPrivateField", "unused"})
  private final Supplier<Optional<Translation2d>> visionTargetSupplier;

  /** The set of valid superstructure states. */
  public enum SuperstructureState {
    STOWED,
    INTAKE_DOWN,
    INTAKE_RUNNING,
    SCORE,
    UNJAM,
    BEACHED
  }

  private final MARSStateMachine<SuperstructureState> stateMachine;
  private int gamePieceCount = 0;

  private double goalCowlAngle = 0.0;
  private double goalIntakeAngle = 0.0;

  private final Supplier<Double> tiltRadiansSupplier;

  /**
   * Constructs the superstructure orchestrator.
   *
   * @param cowl The cowl (hood) rotary mechanism.
   * @param intakePivot The intake pivot rotary mechanism.
   * @param floorIntake The floor intake flywheel (as MARSShooter).
   * @param shooter The main shooter flywheel.
   * @param feeder The feeder flywheel.
   * @param poseSupplier Supplier for the current robot field pose.
   * @param visionTargetSupplier Supplier for the latest vision target translation.
   * @param tiltRadiansSupplier Supplier for the current robot tilt in radians.
   */
  public MARSSuperstructure(
      MARSCowl cowl,
      MARSIntakePivot intakePivot,
      MARSShooter floorIntake,
      MARSShooter shooter,
      MARSShooter feeder,
      Supplier<Pose2d> poseSupplier,
      Supplier<Optional<Translation2d>> visionTargetSupplier,
      Supplier<Double> tiltRadiansSupplier) {

    this.cowl = cowl;
    this.intakePivot = intakePivot;
    this.floorIntake = floorIntake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.poseSupplier = poseSupplier;
    this.visionTargetSupplier = visionTargetSupplier;
    this.tiltRadiansSupplier = tiltRadiansSupplier;

    stateMachine =
        new MARSStateMachine<>(
            "Superstructure", SuperstructureState.class, SuperstructureState.STOWED);

    stateMachine.addValidBidirectional(SuperstructureState.STOWED, SuperstructureState.INTAKE_DOWN);
    stateMachine.addValidBidirectional(
        SuperstructureState.INTAKE_DOWN, SuperstructureState.INTAKE_RUNNING);
    stateMachine.addValidBidirectional(SuperstructureState.STOWED, SuperstructureState.SCORE);
    stateMachine.addValidBidirectional(SuperstructureState.STOWED, SuperstructureState.UNJAM);
    stateMachine.addValidBidirectional(SuperstructureState.STOWED, SuperstructureState.BEACHED);
    stateMachine.addValidBidirectional(
        SuperstructureState.INTAKE_DOWN, SuperstructureState.BEACHED);
    stateMachine.addValidBidirectional(
        SuperstructureState.INTAKE_RUNNING, SuperstructureState.BEACHED);
    stateMachine.addValidBidirectional(SuperstructureState.SCORE, SuperstructureState.BEACHED);
    stateMachine.addValidBidirectional(SuperstructureState.UNJAM, SuperstructureState.BEACHED);

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

  /**
   * Creates a command that requests a transition to the given superstructure state. If the
   * transition is illegal, it is rejected and logged — no mechanism movement occurs.
   *
   * @param targetState The desired superstructure state.
   * @return A command that executes the transition request.
   */
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

  /**
   * Forces the superstructure into the specified state by routing through STOWED first. This
   * bypasses normal transition validation and should only be used for safety overrides (e.g.,
   * automatic beaching on tilt detection).
   *
   * @param targetState The state to force.
   */
  public void forceState(SuperstructureState targetState) {
    stateMachine.requestTransition(SuperstructureState.STOWED);
    stateMachine.requestTransition(targetState);
  }

  @Override
  public void periodic() {
    // Safety: auto-beach when robot tilts beyond 25°
    double tiltDegrees = Math.toDegrees(Math.abs(tiltRadiansSupplier.get()));
    if (tiltDegrees > 25.0 && stateMachine.getState() != SuperstructureState.BEACHED) {
      forceState(SuperstructureState.BEACHED);
    }

    stateMachine.update();
    SuperstructureState currentState = stateMachine.getState();

    // Update mechanism targets based on current state
    updateMechanismTargets(currentState);
    cowl.setTargetPosition(goalCowlAngle);
    intakePivot.setTargetPosition(goalIntakeAngle);

    // Execute state-specific motor logic
    handleIntakeLogic(currentState);
    handleScoringLogic(currentState);
    handleUnjamLogic(currentState);

    // Log state
    logOutputs(currentState);
  }

  /** Sets goalCowlAngle and goalIntakeAngle based on the current state. */
  private void updateMechanismTargets(SuperstructureState currentState) {
    switch (currentState) {
      case INTAKE_DOWN:
      case INTAKE_RUNNING:
        goalIntakeAngle = SuperstructureConstants.INTAKE_PIVOT_ANGLE;
        goalCowlAngle = 0.0;
        break;
      case SCORE:
        goalIntakeAngle = 0.0;
        EliteShooterMath.EliteShooterSetpoint shot = calculateStaticShot();
        if (shot.isValid) {
          goalCowlAngle = shot.hoodRadians;
        } else {
          goalCowlAngle = SuperstructureConstants.SCORE_FALLBACK_COWL_ANGLE;
        }
        break;
      case STOWED:
      case UNJAM:
      case BEACHED:
      default:
        goalIntakeAngle = 0.0;
        goalCowlAngle = 0.0;
        break;
    }
  }

  /** Handles intake roller activation and physics-based game piece collection. */
  private void handleIntakeLogic(SuperstructureState currentState) {
    // Physics collision check for game piece swallowing
    if (currentState == SuperstructureState.INTAKE_RUNNING && gamePieceCount < 40) {
      if (intakePivot.isAtTolerance()) {
        try {
          int swallowed =
              com.marslib.simulation.MARSPhysicsWorld.getInstance()
                  .checkIntake(
                      poseSupplier.get(),
                      FieldConstants.INTAKE_COLLECTION_RADIUS_METERS,
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
  }

  /** Handles shooter spin-up, flywheel readiness checks, feeding, and game piece launching. */
  private void handleScoringLogic(SuperstructureState currentState) {
    if (currentState == SuperstructureState.SCORE) {
      EliteShooterMath.EliteShooterSetpoint shot = calculateStaticShot();
      double targetRadPerSec = 4000.0 * Math.PI * 2.0 / 60.0; // Default fallback
      if (shot.isValid) {
        targetRadPerSec = shot.launchSpeedMetersPerSec * 30.0;
      }

      shooter.setClosedLoopVelocity(targetRadPerSec);

      // Wait for flywheel and cowl to be at tolerance before transferring
      if (shooter.isAtTolerance() && cowl.isAtTolerance()) {
        feeder.setVoltage(12.0);
        floorIntake.setVoltage(12.0);
        launchGamePiece();
      } else {
        feeder.setVoltage(0.0);
        floorIntake.setVoltage(0.0);
      }
    } else if (currentState != SuperstructureState.UNJAM
        && currentState != SuperstructureState.BEACHED) {
      // Manage idle flywheel speed when not scoring
      double currentRPM = shooter.getVelocityRadPerSec() * 60.0 / (Math.PI * 2.0);
      if (currentRPM > 1600.0) {
        shooter.setVoltage(0.0); // Coast down via friction
      } else {
        double idleRadPerSec = 1500.0 * Math.PI * 2.0 / 60.0;
        shooter.setClosedLoopVelocity(idleRadPerSec); // Maintain idle speed
      }
      feeder.setVoltage(0.0);
    } else if (currentState == SuperstructureState.BEACHED) {
      shooter.setVoltage(0.0);
      feeder.setVoltage(0.0);
    }
  }

  /** Handles reverse motor operation for unjamming game pieces. */
  private void handleUnjamLogic(SuperstructureState currentState) {
    if (currentState == SuperstructureState.UNJAM) {
      feeder.setVoltage(-6.0);
      floorIntake.setVoltage(-6.0);
      shooter.setVoltage(-6.0);
    }
  }

  /**
   * Calculates a static (zero-velocity) shot setpoint to the alliance-appropriate hub using {@link
   * EliteShooterMath}.
   */
  private EliteShooterMath.EliteShooterSetpoint calculateStaticShot() {
    Translation2d hub =
        AllianceUtil.isRed() ? FieldConstants.RED_HUB_POS : FieldConstants.BLUE_HUB_POS;

    return EliteShooterMath.calculateShotOnTheMove(
        poseSupplier.get(),
        new ChassisSpeeds(),
        new Translation3d(hub.getX(), hub.getY(), FieldConstants.HUB_SIZE_METERS),
        FieldConstants.GAME_PIECE_REST_HEIGHT_METERS,
        ShooterConstants.PROJECTILE_SPEED_MPS,
        -9.81,
        0.1);
  }

  /** Launches a game piece from the robot in simulation, validating scoring zone compliance. */
  private void launchGamePiece() {
    if (gamePieceCount > 0) {
      Pose2d robotPose = poseSupplier.get();
      boolean isBlue = AllianceUtil.isBlue();

      // Ensure on the correct side (182.11 inches or 4.625594 m from end of field)
      double allowedDistance = 4.625594;
      boolean correctSide =
          isBlue
              ? (robotPose.getX() <= allowedDistance)
              : (robotPose.getX() >= FieldConstants.FIELD_LENGTH_METERS - allowedDistance);

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
              SuperstructureConstants.SHOOTER_EXIT_HEIGHT_METERS,
              correctSide);
      gamePieceCount--;
    }
  }

  /** Logs all superstructure telemetry outputs. */
  private void logOutputs(SuperstructureState currentState) {
    Logger.recordOutput("Superstructure/GamePieceCount", gamePieceCount);
    Logger.recordOutput("Superstructure/GoalCowlAngle", goalCowlAngle);
    Logger.recordOutput("Superstructure/GoalIntakeAngle", goalIntakeAngle);
    Logger.recordOutput("Superstructure/CurrentState", currentState.name());
  }

  /**
   * Returns the current superstructure state.
   *
   * @return The active {@link SuperstructureState}.
   */
  public SuperstructureState getCurrentState() {
    return stateMachine.getState();
  }

  /**
   * Returns the underlying state machine for testing and diagnostics.
   *
   * @return The {@link MARSStateMachine} instance.
   */
  public MARSStateMachine<SuperstructureState> getStateMachine() {
    return stateMachine;
  }
}
