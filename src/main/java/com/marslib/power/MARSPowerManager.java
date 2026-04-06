package com.marslib.power;

import com.marslib.faults.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem responsible for monitoring the central Power Distribution Hub (PDH).
 *
 * <p>Students: This layer pulls input telemetry directly from the physical hardware
 * (Voltage/Current) and actively manages structural alerts. Most importantly, it pushes real-time
 * voltage down into the SwerveDrive subsystem allowing the robot to automatically execute "Load
 * Shedding" to prevent brownouts.
 */
public class MARSPowerManager extends SubsystemBase {
  private final PowerIO io;
  private final PowerIOInputsAutoLogged inputs = new PowerIOInputsAutoLogged();

  private final Alert warningAlert =
      new Alert(
          "Power",
          "Load Shedding: Voltage below " + Constants.PowerConstants.WARNING_VOLTAGE + "V",
          Alert.AlertType.WARNING);
  private final Alert criticalAlert =
      new Alert(
          "Power",
          "Load Shedding: Voltage below " + Constants.PowerConstants.CRITICAL_VOLTAGE + "V",
          Alert.AlertType.CRITICAL);

  /**
   * Initializes the Power Manager.
   *
   * @param io The selected IO layer (Sim or Hardware PDH) pulling raw voltages.
   */
  public MARSPowerManager(PowerIO io) {
    this.io = io;
  }

  /**
   * Processes IO loops periodically. It triggers AdvantageScope "Alerts" if voltage falls below
   * safe structural operating limits.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Power", inputs);

    if (inputs.voltage > 0) {
      if (inputs.voltage < Constants.PowerConstants.CRITICAL_VOLTAGE) {
        warningAlert.set(true);
        criticalAlert.set(true);
      } else if (inputs.voltage < Constants.PowerConstants.WARNING_VOLTAGE) {
        warningAlert.set(true);
        criticalAlert.set(false);
      } else {
        warningAlert.set(false);
        criticalAlert.set(false);
      }
    } else {
      warningAlert.set(false);
      criticalAlert.set(false);
    }
  }

  /**
   * Evaluates the absolute bus voltage dynamically.
   *
   * @return The exact system voltage across the PDP/PDH. Usually ~12.5V, dropping during heavy
   *     loads.
   */
  public double getVoltage() {
    return inputs.voltage;
  }
}
