package com.marslib.hmi;

import static frc.robot.constants.ModeConstants.*;

import com.marslib.faults.MARSFaultManager;
import com.marslib.power.MARSPowerManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.PowerConstants;

/** Centralized manager for robot LED logic, translating robot states into visual feedback. */
public class LEDManager extends SubsystemBase {
  private final LEDIO io;
  private final MARSPowerManager powerManager;

  public LEDManager(LEDIO io, MARSPowerManager powerManager) {
    this.io = io;
    this.powerManager = powerManager;
  }

  @Override
  public void periodic() {
    if (MARSFaultManager.hasActiveCriticalFaults()) {
      io.setCriticalFaultFlash();
    } else if (powerManager.getVoltage() < PowerConstants.WARNING_VOLTAGE) {
      io.setLoadSheddingColors();
    } else {
      io.setDefaultColors();
    }

    io.update();
  }
}
