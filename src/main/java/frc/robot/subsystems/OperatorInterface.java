package frc.robot.subsystems;

import com.marslib.faults.MARSFaultManager;
import com.marslib.hmi.TelemetryGamepad;
import com.marslib.mechanisms.*;
import com.marslib.power.MARSPowerManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem handling driver and operator input, including intelligent rumble feedback for faults
 * and voltage sags.
 */
public class OperatorInterface extends SubsystemBase {
  private final TelemetryGamepad controller;
  private final MARSPowerManager powerManager;

  // Pulse parameters
  private boolean isPulsing = false;
  private int pulseCounter = 0;
  private static final int PULSE_DURATION_LOOPS =
      frc.robot.constants.OperatorConstants.PULSE_DURATION_LOOPS;
  private static final int PULSE_INTERVAL_LOOPS =
      frc.robot.constants.OperatorConstants.PULSE_INTERVAL_LOOPS;

  public OperatorInterface(int port, MARSPowerManager powerManager) {
    this.controller = new TelemetryGamepad(port, "DrivePilot");
    this.powerManager = powerManager;
  }

  public TelemetryGamepad getController() {
    return controller;
  }

  public void setRumble(double strength) {
    controller.getHID().setRumble(RumbleType.kBothRumble, strength);
  }

  @Override
  public void periodic() {
    // 1. Critical Fault Pulse Priority
    if (MARSFaultManager.hasNewCriticalFault()) {
      isPulsing = true;
      pulseCounter = 0;
      MARSFaultManager.clearNewCriticalFault();
    }

    if (isPulsing) {
      if (pulseCounter < PULSE_DURATION_LOOPS) {
        setRumble(1.0);
      } else if (pulseCounter < PULSE_DURATION_LOOPS + PULSE_INTERVAL_LOOPS) {
        setRumble(0.0);
      } else {
        isPulsing = false;
        setRumble(0.0);
      }
      pulseCounter++;
      return; // Skip voltage rumble if critical fault is pulsing
    }

    // 2. Voltage Droop Rumble
    double voltage = powerManager.getVoltage();
    double nominal = frc.robot.constants.PowerConstants.NOMINAL_VOLTAGE;
    double critical = frc.robot.constants.PowerConstants.CRITICAL_VOLTAGE;

    if (voltage > 0.0 && voltage < nominal) {
      double voltageRange = nominal - critical;
      double rumbleStrength = 1.0 - ((voltage - critical) / voltageRange);
      rumbleStrength = Math.max(0.0, Math.min(1.0, rumbleStrength));
      setRumble(rumbleStrength);
    } else {
      setRumble(0.0);
    }
  }
}
