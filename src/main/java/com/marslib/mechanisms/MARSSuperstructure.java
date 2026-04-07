package com.marslib.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Superstructure scheduler mapping safe collision logic between the Elevator and the Arm. */
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

  private SuperstructureState currentState = SuperstructureState.STOWED;
  private boolean hasPiece = false;

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
  }

  // Idealistic goal states
  private double goalElevatorHeight = 0.0;
  private double goalArmAngle = 0.0;

  /**
   * Commands a transition to the specified superstructure state.
   *
   * @param targetState The desired {@link SuperstructureState}.
   * @return An instant command that sets the active state.
   */
  public Command setAbsoluteState(SuperstructureState targetState) {
    return Commands.runOnce(() -> currentState = targetState, this);
  }

  @Override
  public void periodic() {
    // 1. Map enum state to idealistic spatial goals
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

    // 2. Safe collision bounding logic (2D State Machine constraint clamping)
    double safeArmAngle = goalArmAngle;
    double safeElevatorHeight = goalElevatorHeight;

    // RULE A: If elevator is physically low, the arm is not allowed to extend outwards.
    // This prevents the arm from smashing into the floor/bumpers.
    if (elevator.getPositionMeters()
        < frc.robot.Constants.SuperstructureConstants.SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW) {
      // Clamp arm to a safe stowed boundary when elevator is down
      safeArmAngle =
          Math.min(
              safeArmAngle,
              frc.robot.Constants.SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MAX_STOW);
    }

    // RULE B: If the arm is currently extended outside the safe stow bounds,
    // the elevator is not allowed to drop below the safe transit height.
    if (arm.getPositionRads()
        > frc.robot.Constants.SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MIN_EXTEND) {
      // Clamp elevator minimum height to prevent it crushing downwards while arm is out
      safeElevatorHeight =
          Math.max(
              safeElevatorHeight,
              frc.robot.Constants.SuperstructureConstants.SAFE_ELEVATOR_HEIGHT_METERS_MIN);
    }

    // 3. Dispatch safe commands continuously
    elevator.setTargetPosition(safeElevatorHeight);
    arm.setTargetPosition(safeArmAngle);

    // 4. Intake logic — works in both Sim and Replay by guarding on physics world availability
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
        // Physics world not available (Real hardware) — intake detection handled by sensor IO
      }
    } else if (currentState == SuperstructureState.SCORE_HIGH) {
      hasPiece = false; // Dump piece when scoring
    }

    // Abstract hardware integration
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

    Logger.recordOutput("Superstructure/HasPiece", hasPiece);
    Logger.recordOutput("Superstructure/CurrentState", currentState.name());
    Logger.recordOutput("Superstructure/GoalElevatorHeight", goalElevatorHeight);
    Logger.recordOutput("Superstructure/GoalArmAngle", goalArmAngle);
    Logger.recordOutput("Superstructure/SafeElevatorHeight", safeElevatorHeight);
    Logger.recordOutput("Superstructure/SafeArmAngle", safeArmAngle);
  }

  /**
   * Returns the currently active superstructure state.
   *
   * @return The current {@link SuperstructureState}.
   */
  public SuperstructureState getCurrentState() {
    return currentState;
  }
}
