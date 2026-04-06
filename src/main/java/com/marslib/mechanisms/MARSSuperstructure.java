package com.marslib.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Superstructure scheduler mapping safe collision logic between the Elevator and the Arm. */
public class MARSSuperstructure extends SubsystemBase {

  private final MARSElevator elevator;
  private final MARSArm arm;

  public enum SuperstructureState {
    STOWED,
    INTAKE_FLOOR,
    SCORE_HIGH
  }

  private SuperstructureState currentState = SuperstructureState.STOWED;

  public MARSSuperstructure(MARSElevator elevator, MARSArm arm) {
    this.elevator = elevator;
    this.arm = arm;
  }

  // Idealistic goal states
  private double goalElevatorHeight = 0.0;
  private double goalArmAngle = 0.0;

  /** Updates the target state of the superstructure dynamically. */
  public Command setAbsoluteState(SuperstructureState targetState) {
    return Commands.runOnce(() -> currentState = targetState, this);
  }

  @Override
  public void periodic() {
    // 1. Map enum state to idealistic spatial goals
    switch (currentState) {
      case INTAKE_FLOOR:
        goalElevatorHeight = 0.2;
        goalArmAngle = Math.PI / 4;
        break;
      case SCORE_HIGH:
        goalElevatorHeight = 1.5;
        goalArmAngle = Math.PI / 2;
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
    if (elevator.getPositionMeters() < 0.5) {
      // Clamp arm to a safe stowed boundary when elevator is down
      safeArmAngle = Math.min(safeArmAngle, 0.2);
    }

    // RULE B: If the arm is currently extended outside the safe stow bounds,
    // the elevator is not allowed to drop below the safe transit height.
    if (arm.getPositionRads() > 0.3) {
      // Clamp elevator minimum height to prevent it crushing downwards while arm is out
      safeElevatorHeight = Math.max(safeElevatorHeight, 0.6);
    }

    // 3. Dispatch safe commands continuously
    elevator.setTargetPosition(safeElevatorHeight);
    arm.setTargetPosition(safeArmAngle);
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }
}
