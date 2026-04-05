package com.marslib.hmi;

import com.marslib.faults.MARSFaultManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDManager extends SubsystemBase {
  private final LEDIO io;

  public LEDManager(LEDIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if (MARSFaultManager.hasActiveCriticalFaults()) {
      io.setCriticalFaultFlash();
    } else {
      io.setDefaultColors();
    }

    io.update();
  }
}
