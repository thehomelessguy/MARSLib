package com.marslib.mechanisms;

import com.marslib.power.MARSPowerManager;
import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MARSElevator extends SubsystemBase {

  private final LinearMechanismIO io;
  private final LinearMechanismIOInputsAutoLogged inputs = new LinearMechanismIOInputsAutoLogged();

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);

  private ElevatorFeedforward feedforward;

  // Bounds for active load shedding
  private static final double NOMINAL_VOLTAGE = 10.0;
  private static final double CRITICAL_VOLTAGE = 7.0;

  private static final double MAX_CURRENT_AMPS = 60.0;
  private static final double MIN_CURRENT_AMPS = 20.0;

  private double currentTargetMeters = 0.0;
  private final MARSPowerManager powerManager;

  public MARSElevator(LinearMechanismIO io, MARSPowerManager powerManager) {
    this.io = io;
    this.powerManager = powerManager;
    feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update Feedforward if TUNING mode constants are changed
    if (kS.hasChanged(kS.hashCode())
        || kG.hasChanged(kG.hashCode())
        || kV.hasChanged(kV.hashCode())
        || kA.hasChanged(kA.hashCode())) {
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

  public void setTargetPosition(double positionMeters) {
    currentTargetMeters = positionMeters;
    double ffVolts = feedforward.calculate(0.0); // Simple static FF
    io.setClosedLoopPosition(positionMeters, ffVolts);
    Logger.recordOutput("Elevator/TargetPositionMeters", positionMeters);
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }
}
