package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Domain-specific wrapper for MARSLib climb mechanism. Uses composition to wrap the generic
 * MARSElevator and potentially RotaryMechanism for flips.
 */
public class MARSClimber extends SubsystemBase {

  private final MARSElevator fastClimber;
  private final MARSArm flipClimber;

  public MARSClimber(MARSElevator fastClimber) {
    this(fastClimber, null);
  }

  public MARSClimber(MARSElevator fastClimber, MARSArm flipClimber) {
    this.fastClimber = fastClimber;
    this.flipClimber = flipClimber;
  }

  public void setFastClimbVoltage(double volts) {
    fastClimber.setVoltage(volts);
  }

  public void setFastClimbPosition(double positionMeters) {
    fastClimber.setTargetPosition(positionMeters);
  }

  public double getFastClimbPosition() {
    return fastClimber.getPositionMeters();
  }

  public void setFlipClimbPosition(double positionRads) {
    if (flipClimber != null) {
      flipClimber.setTargetPosition(positionRads);
    }
  }

  @Override
  public void periodic() {
    // Top-level telemetry could be aggregated here if needed
  }
}
