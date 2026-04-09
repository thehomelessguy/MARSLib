package com.marslib.power;

import static frc.robot.constants.ModeConstants.*;

import com.marslib.faults.Alert;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.PowerConstants;
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
          "Load Shedding: Voltage below " + PowerConstants.WARNING_VOLTAGE + "V",
          Alert.AlertType.WARNING);
  private final Alert criticalAlert =
      new Alert(
          "Power",
          "Load Shedding: Voltage below " + PowerConstants.CRITICAL_VOLTAGE + "V",
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
      if (inputs.voltage < PowerConstants.CRITICAL_VOLTAGE) {
        warningAlert.set(true);
        criticalAlert.set(true);
      } else if (inputs.voltage < PowerConstants.WARNING_VOLTAGE) {
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

  /**
   * Helper function for mechanisms to compute their dynamically shedded current limit based on bus
   * voltage. Linearly scales current limits back when voltage sags between NOMINAL and CRITICAL.
   */
  public double calculateLoadSheddedLimit(
      double maxCurrent, double minCurrent, double nominalVoltage, double criticalVoltage) {
    if (inputs.voltage >= nominalVoltage) {
      return maxCurrent;
    }
    double slope = (maxCurrent - minCurrent) / (nominalVoltage - criticalVoltage);
    double limit = minCurrent + slope * (inputs.voltage - criticalVoltage);
    return MathUtil.clamp(limit, minCurrent, maxCurrent);
  }
}
