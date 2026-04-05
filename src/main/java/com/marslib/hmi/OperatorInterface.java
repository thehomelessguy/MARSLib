package com.marslib.hmi;

import com.marslib.faults.MARSFaultManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorInterface extends SubsystemBase {
  private final CommandXboxController controller;

  // Pulse parameters
  private boolean isPulsing = false;
  private int pulseCounter = 0;
  private static final int PULSE_DURATION_LOOPS = 25; // 500ms
  private static final int PULSE_INTERVAL_LOOPS = 25; // 500ms

  public OperatorInterface(int port) {
    controller = new CommandXboxController(port);
  }

  public CommandXboxController getController() {
    return controller;
  }

  public void setRumble(double strength) {
    controller.getHID().setRumble(RumbleType.kBothRumble, strength);
  }

  @Override
  public void periodic() {
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
    }
  }
}
