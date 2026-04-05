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

  /** Generates a collision-safe dynamic Command grouping to transition to the required state. */
  public Command setAbsoluteState(SuperstructureState targetState) {
    return Commands.runOnce(() -> currentState = targetState)
        .andThen(
            Commands.defer(
                () -> {
                  switch (targetState) {
                    case INTAKE_FLOOR:
                      // Safe move: Arm down first, then elevator out slightly
                      return Commands.sequence(
                          arm.runOnce(() -> arm.setTargetPosition(Math.PI / 4)),
                          Commands.waitUntil(
                              () -> Math.abs(arm.getPositionRads() - Math.PI / 4) < 0.1),
                          elevator.runOnce(() -> elevator.setTargetPosition(0.2)));

                    case SCORE_HIGH:
                      // Safe move: Elevator up first, then Arm out to score
                      return Commands.sequence(
                          elevator.runOnce(() -> elevator.setTargetPosition(1.5)),
                          Commands.waitUntil(
                              () -> Math.abs(elevator.getPositionMeters() - 1.5) < 0.1),
                          arm.runOnce(() -> arm.setTargetPosition(Math.PI / 2)));

                    case STOWED:
                    default:
                      // Safe move: Arm heavily stowed *first* to avoid smashing chassis, then
                      // elevator drops
                      return Commands.sequence(
                          arm.runOnce(() -> arm.setTargetPosition(0.0)),
                          Commands.waitUntil(() -> Math.abs(arm.getPositionRads()) < 0.2),
                          elevator.runOnce(() -> elevator.setTargetPosition(0.0)));
                  }
                },
                java.util.Set.of(this)));
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }
}
