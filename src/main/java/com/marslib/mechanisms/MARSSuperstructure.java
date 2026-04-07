package com.marslib.mechanisms;

import com.marslib.util.MARSStateMachine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Collision-safe superstructure orchestrator using a validated finite state machine.
 *
 * <p>This subsystem coordinates the elevator, arm, intake, and shooter mechanisms through a {@link
 * MARSStateMachine} that enforces legal state transitions and logs every decision. The collision
 * safety rules (arm-lock when elevator is low, elevator-hold when arm is extended) are applied as
 * continuous constraint clamping on top of the FSM's goal positions.
 *
 * <p><b>Key design:</b> The FSM validates transitions (e.g., you cannot go directly from
 * INTAKE_FLOOR → SCORE_HIGH without passing through STOWED). Illegal transitions are rejected and
 * logged, giving the drive team clear diagnostic data in AdvantageScope.
 */
public class MARSSuperstructure extends SubsystemBase {

  private final MARSElevator elevator;
  private final MARSArm arm;
  private final MARSIntake intake;
  private final MARSShooter shooter;
  private final java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier;

  /** Discrete states representing the superstructure's target physical configuration. */
  public enum SuperstructureState {
    /** Elevator fully retracted, arm stowed — safe for transit and defense. */
    STOWED,
    /** Elevator low, arm extended to floor intake angle. */
    INTAKE_FLOOR,
    /** Elevator fully extended, arm at high scoring angle. */
    SCORE_HIGH
  }

  private final MARSStateMachine<SuperstructureState> stateMachine;
  private boolean hasPiece = false;

  // Idealistic goal states (set by the state machine's current state)
  private double goalElevatorHeight = 0.0;
  private double goalArmAngle = 0.0;

  public MARSSuperstructure(
      MARSElevator elevator,
      MARSArm arm,
      MARSIntake intake,
      MARSShooter shooter,
      java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.shooter = shooter;
    this.poseSupplier = poseSupplier;

    // Build the state machine with validated transitions
    stateMachine =
        new MARSStateMachine<>(
            "Superstructure", SuperstructureState.class, SuperstructureState.STOWED);

    // Define legal transitions:
    //   STOWED can go to anything (it's the safe home state)
    stateMachine.addWildcardFrom(SuperstructureState.STOWED);

    //   From INTAKE_FLOOR, you can stow or go score
    stateMachine.addTransition(SuperstructureState.INTAKE_FLOOR, SuperstructureState.STOWED);
    stateMachine.addTransition(SuperstructureState.INTAKE_FLOOR, SuperstructureState.SCORE_HIGH);

    //   From SCORE_HIGH, you must return to STOWED (no direct intake transition)
    stateMachine.addTransition(SuperstructureState.SCORE_HIGH, SuperstructureState.STOWED);

    // Entry actions — log what we're doing when entering each state
    stateMachine.setEntryAction(
        SuperstructureState.SCORE_HIGH,
        () -> {
          hasPiece = false; // Piece ejected when entering SCORE
          Logger.recordOutput("Superstructure/EntryAction", "SCORE_HIGH: piece ejected");
        });

    // Transition callback for detailed audit trail
    stateMachine.setOnTransition(
        (from, to) ->
            Logger.recordOutput(
                "Superstructure/TransitionDetail",
                String.format(
                    "%s→%s at elevPos=%.3fm armPos=%.3frad hasPiece=%b",
                    from.name(),
                    to.name(),
                    elevator.getPositionMeters(),
                    arm.getPositionRads(),
                    hasPiece)));
  }

  /**
   * Commands a transition to the specified superstructure state. If the transition is illegal
   * (e.g., SCORE_HIGH → INTAKE_FLOOR), it is rejected and logged — the superstructure stays in its
   * current state.
   *
   * @param targetState The desired {@link SuperstructureState}.
   * @return An instant command that requests the transition.
   */
  public Command setAbsoluteState(SuperstructureState targetState) {
    return Commands.runOnce(
        () -> {
          boolean accepted = stateMachine.requestTransition(targetState);
          if (!accepted) {
            Logger.recordOutput(
                "Superstructure/TransitionRejectedReason",
                String.format(
                    "%s→%s is not a legal transition. Must go through STOWED first.",
                    stateMachine.getState().name(), targetState.name()));
          }
        },
        this);
  }

  /**
   * Forcibly sets the state without transition validation. Use only in emergency scenarios (e.g.,
   * e-stop recovery, test setup). Illegal in competition code.
   *
   * @param targetState The state to force.
   */
  public void forceState(SuperstructureState targetState) {
    // Use the wildcard-to pattern to ensure STOWED is always reachable
    stateMachine.requestTransition(SuperstructureState.STOWED);
    stateMachine.requestTransition(targetState);
  }

  @Override
  public void periodic() {
    // 0. Update the state machine tick counter and log current state
    stateMachine.update();

    SuperstructureState currentState = stateMachine.getState();

    // 1. Map current state to goal positions
    switch (currentState) {
      case INTAKE_FLOOR:
        goalElevatorHeight = frc.robot.Constants.SuperstructureConstants.INTAKE_ELEVATOR_HEIGHT;
        goalArmAngle = frc.robot.Constants.SuperstructureConstants.INTAKE_ARM_ANGLE;
        break;
      case SCORE_HIGH:
        goalElevatorHeight = frc.robot.Constants.SuperstructureConstants.SCORE_HIGH_ELEVATOR_HEIGHT;
        goalArmAngle = frc.robot.Constants.SuperstructureConstants.SCORE_HIGH_ARM_ANGLE;
        break;
      case STOWED:
      default:
        goalElevatorHeight = 0.0;
        goalArmAngle = 0.0;
        break;
    }

    // 2. Collision constraint clamping
    double safeArmAngle = goalArmAngle;
    double safeElevatorHeight = goalElevatorHeight;

    // RULE A: Arm-lock when elevator is low
    boolean armClamped = false;
    if (elevator.getPositionMeters()
        < frc.robot.Constants.SuperstructureConstants.SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW) {
      double clampedArm =
          Math.min(
              safeArmAngle,
              frc.robot.Constants.SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MAX_STOW);
      if (clampedArm != safeArmAngle) {
        armClamped = true;
        Logger.recordOutput(
            "Superstructure/CollisionClamp",
            String.format(
                "ARM_LOCKED: goal=%.2f clamped=%.2f (elevator at %.2fm < %.2fm threshold)",
                safeArmAngle,
                clampedArm,
                elevator.getPositionMeters(),
                frc.robot.Constants.SuperstructureConstants.SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW));
      }
      safeArmAngle = clampedArm;
    }

    // RULE B: Elevator-hold when arm is extended
    boolean elevatorClamped = false;
    if (arm.getPositionRads()
        > frc.robot.Constants.SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MIN_EXTEND) {
      double clampedElevator =
          Math.max(
              safeElevatorHeight,
              frc.robot.Constants.SuperstructureConstants.SAFE_ELEVATOR_HEIGHT_METERS_MIN);
      if (clampedElevator != safeElevatorHeight) {
        elevatorClamped = true;
        Logger.recordOutput(
            "Superstructure/CollisionClamp",
            String.format(
                "ELEVATOR_HELD: goal=%.2f clamped=%.2f (arm at %.2frad > %.2frad threshold)",
                safeElevatorHeight,
                clampedElevator,
                arm.getPositionRads(),
                frc.robot.Constants.SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MIN_EXTEND));
      }
      safeElevatorHeight = clampedElevator;
    }

    // 3. Dispatch safe commands
    elevator.setTargetPosition(safeElevatorHeight);
    arm.setTargetPosition(safeArmAngle);

    // 4. Intake logic
    if (currentState == SuperstructureState.INTAKE_FLOOR && !hasPiece) {
      try {
        boolean swallow =
            com.marslib.simulation.MARSPhysicsWorld.getInstance()
                .checkIntake(
                    poseSupplier.get(),
                    frc.robot.Constants.FieldConstants.INTAKE_COLLECTION_RADIUS_METERS);
        if (swallow) {
          hasPiece = true;
        }
      } catch (Exception e) {
        // Physics world not available (Real hardware)
      }
    }

    // 5. Actuator control
    if (currentState == SuperstructureState.INTAKE_FLOOR && !hasPiece) {
      intake.setVoltage(12.0);
    } else {
      intake.setVoltage(0.0);
    }

    if (currentState == SuperstructureState.SCORE_HIGH) {
      shooter.setClosedLoopVelocity(400.0, 0.0);
    } else {
      shooter.setVoltage(0.0);
    }

    // 6. Telemetry
    Logger.recordOutput("Superstructure/HasPiece", hasPiece);
    Logger.recordOutput("Superstructure/GoalElevatorHeight", goalElevatorHeight);
    Logger.recordOutput("Superstructure/GoalArmAngle", goalArmAngle);
    Logger.recordOutput("Superstructure/SafeElevatorHeight", safeElevatorHeight);
    Logger.recordOutput("Superstructure/SafeArmAngle", safeArmAngle);
    Logger.recordOutput("Superstructure/ArmClamped", armClamped);
    Logger.recordOutput("Superstructure/ElevatorClamped", elevatorClamped);
  }

  /** Returns the currently active superstructure state. */
  public SuperstructureState getCurrentState() {
    return stateMachine.getState();
  }

  /** Returns the underlying state machine for advanced inspection (e.g., testing). */
  public MARSStateMachine<SuperstructureState> getStateMachine() {
    return stateMachine;
  }
}
