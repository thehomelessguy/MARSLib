package com.marslib.hmi;

import com.marslib.faults.MARSFaultManager;
import com.marslib.power.MARSPowerManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorInterface extends SubsystemBase {
  private final CommandXboxController controller;
  private final MARSPowerManager powerManager;

  // Pulse parameters
  private boolean isPulsing = false;
  private int pulseCounter = 0;
  private static final int PULSE_DURATION_LOOPS = 25; // 500ms
  private static final int PULSE_INTERVAL_LOOPS = 25; // 500ms

  public OperatorInterface(int port, MARSPowerManager powerManager) {
    this.controller = new CommandXboxController(port);
    this.powerManager = powerManager;
  }

  public CommandXboxController getController() {
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
    if (voltage > 0.0 && voltage < 10.0) {
      // Map 10V to 0.0 strength and 7.0V to 1.0 strength
      double rumbleStrength = 1.0 - ((voltage - 7.0) / 3.0);
      rumbleStrength = Math.max(0.0, Math.min(1.0, rumbleStrength));
      setRumble(rumbleStrength);
    } else {
      setRumble(0.0);
    }
  }
}
