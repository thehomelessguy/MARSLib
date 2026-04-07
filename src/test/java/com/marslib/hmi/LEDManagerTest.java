package com.marslib.hmi;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.faults.Alert;
import com.marslib.faults.MARSFaultManager;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for LEDManager state-driven routing.
 *
 * <p>Uses a recording LEDIO implementation to verify that the correct LED method is called based on
 * fault and power conditions.
 */
public class LEDManagerTest {

  /** Records which LED method was last called. */
  private static class RecordingLEDIO implements LEDIO {
    String lastCall = "";

    @Override
    public void setDefaultColors() {
      lastCall = "default";
    }

    @Override
    public void setLoadSheddingColors() {
      lastCall = "loadShedding";
    }

    @Override
    public void setCriticalFaultFlash() {
      lastCall = "criticalFault";
    }

    @Override
    public void update() {}
  }

  private RecordingLEDIO recordingIO;
  private LEDManager ledManager;
  private double spoofedVoltage = 12.0;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    Alert.resetAll();
    MARSFaultManager.clear();

    recordingIO = new RecordingLEDIO();

    PowerIO spoofedPowerIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = spoofedVoltage;
            inputs.isBrownedOut = spoofedVoltage < 6.0;
          }
        };

    MARSPowerManager power = new MARSPowerManager(spoofedPowerIO);
    power.periodic(); // Prime the voltage reading
    ledManager = new LEDManager(recordingIO, power);
  }

  @Test
  public void testDefaultColorsUnderNormalVoltage() {
    spoofedVoltage = 12.0;
    ledManager.periodic();
    assertEquals("default", recordingIO.lastCall, "Should show default colors at nominal voltage.");
  }

  @Test
  public void testLoadSheddingUnderLowVoltage() {
    spoofedVoltage = Constants.PowerConstants.WARNING_VOLTAGE - 0.5;
    // Must re-prime power manager with new voltage
    PowerIO lowVoltageIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = spoofedVoltage;
          }
        };
    MARSPowerManager lowPower = new MARSPowerManager(lowVoltageIO);
    lowPower.periodic();
    LEDManager lowLedManager = new LEDManager(recordingIO, lowPower);

    lowLedManager.periodic();
    assertEquals("loadShedding", recordingIO.lastCall, "Should show load shedding at low voltage.");
  }

  @Test
  public void testCriticalFaultOverridesVoltage() {
    // Even with normal voltage, a critical fault should take priority
    spoofedVoltage = 12.0;
    MARSFaultManager.reportHardwareDisconnect("TestFault");

    ledManager.periodic();
    assertEquals(
        "criticalFault",
        recordingIO.lastCall,
        "Critical faults must override voltage-based LED state.");
  }
}
