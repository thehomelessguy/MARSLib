package com.marslib.mechanisms;

import static edu.wpi.first.units.Units.Volts;

import com.marslib.power.MARSPowerManager;
import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * High-level subsystem representing a linearly actuating elevator.
 *
 * <p>Automates physics simulation, position tracking, and dynamic current limiting based on system
 * voltage bounds.
 */
public class MARSElevator extends SubsystemBase {

  private final LinearMechanismIO io;
  private final LinearMechanismIOInputsAutoLogged inputs = new LinearMechanismIOInputsAutoLogged();

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);

  private ElevatorFeedforward feedforward;

  // Bounds for active load shedding
  private static final double NOMINAL_VOLTAGE = Constants.ElevatorConstants.NOMINAL_VOLTAGE;
  private static final double CRITICAL_VOLTAGE = Constants.ElevatorConstants.CRITICAL_VOLTAGE;

  private static final double MAX_CURRENT_AMPS = Constants.ElevatorConstants.MAX_CURRENT_AMPS;
  private static final double MIN_CURRENT_AMPS = Constants.ElevatorConstants.MIN_CURRENT_AMPS;

  private final MARSPowerManager powerManager;
  private final SysIdRoutine sysIdRoutine;

  /**
   * Constructs a MARSElevator with the given IO layer and power manager.
   *
   * @param io The mechanism IO layer (TalonFX or simulation).
   * @param powerManager The active power manager for load-shedding voltage queries.
   */
  public MARSElevator(LinearMechanismIO io, MARSPowerManager powerManager) {
    this.io = io;
    this.powerManager = powerManager;
    feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (edu.wpi.first.units.measure.Voltage volts) -> {
                  io.setVoltage(volts.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update Feedforward if TUNING mode constants are changed
    boolean sChanged = kS.hasChanged(kS.hashCode());
    boolean gChanged = kG.hasChanged(kG.hashCode());
    boolean vChanged = kV.hasChanged(kV.hashCode());
    boolean aChanged = kA.hasChanged(kA.hashCode());

    if (sChanged || gChanged || vChanged || aChanged) {
      feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    // Active Load Shedding via MARSPowerManager
    double sysVoltage = powerManager.getVoltage();
    double currentLimit = MAX_CURRENT_AMPS;

    if (sysVoltage < NOMINAL_VOLTAGE) {
      double slope = (MAX_CURRENT_AMPS - MIN_CURRENT_AMPS) / (NOMINAL_VOLTAGE - CRITICAL_VOLTAGE);
      currentLimit = MIN_CURRENT_AMPS + slope * (sysVoltage - CRITICAL_VOLTAGE);
      currentLimit = Math.max(MIN_CURRENT_AMPS, Math.min(MAX_CURRENT_AMPS, currentLimit));
    }

    io.setCurrentLimit(currentLimit);
    Logger.recordOutput("Elevator/ActiveCurrentLimit", currentLimit);
  }

  /**
   * Commands the elevator to a target height using Motion Magic with dynamic feedforward.
   *
   * <p>The feedforward voltage is computed using the instantaneous Motion Magic profile velocity
   * (for kV contribution) and the static gravity term kG.
   *
   * @param positionMeters Target elevator height in meters.
   */
  public void setTargetPosition(double positionMeters) {
    // Dynamic FF using instantaneous profile target velocity from CTRE Motion Magic
    double ffVolts = feedforward.calculate(inputs.targetVelocityMetersPerSec);
    io.setClosedLoopPosition(positionMeters, ffVolts);
    Logger.recordOutput("Elevator/TargetPositionMeters", positionMeters);
  }

  /**
   * Returns the current measured elevator height.
   *
   * @return Current elevator position in meters.
   */
  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  /**
   * Generates a SysId Quasistatic characterization command.
   *
   * @param direction The direction of the quasistatic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Generates a SysId Dynamic characterization command.
   *
   * @param direction The direction of the dynamic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
