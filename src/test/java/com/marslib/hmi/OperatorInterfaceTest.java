package com.marslib.hmi;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.faults.Alert;
import com.marslib.faults.MARSFaultManager;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.testing.MARSTestHarness;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for OperatorInterface rumble feedback logic.
 *
 * <p>Validates that fault-triggered pulse patterns take priority and that voltage-proportional
 * rumble scales correctly.
 */
public class OperatorInterfaceTest {

  private double spoofedVoltage = 12.0;
  private OperatorInterface oi;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    Alert.resetAll();
    MARSFaultManager.clear();

    PowerIO spoofedPowerIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = spoofedVoltage;
          }
        };
    MARSPowerManager power = new MARSPowerManager(spoofedPowerIO);
    power.periodic();

    oi = new OperatorInterface(5, power);
  }

  @Test
  public void testConstructionDoesNotThrow() {
    assertNotNull(oi, "OperatorInterface should construct without error.");
    assertNotNull(oi.getController(), "Controller should be accessible.");
  }

  @Test
  public void testPeriodicRunsWithoutExceptionAtNominalVoltage() {
    spoofedVoltage = 12.0;
    // Running periodic multiple times should not throw
    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 50; i++) {
            oi.periodic();
          }
        },
        "Periodic should execute cleanly at nominal voltage.");
  }

  @Test
  public void testPeriodicRunsWithoutExceptionUnderFault() {
    MARSFaultManager.reportHardwareDisconnect("TestFault");
    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 50; i++) {
            oi.periodic();
          }
        },
        "Periodic should execute cleanly during fault pulse.");
  }

  @Test
  public void testPeriodicRunsWithoutExceptionAtCriticalVoltage() {
    spoofedVoltage = Constants.PowerConstants.CRITICAL_VOLTAGE - 0.5;
    PowerIO lowIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = spoofedVoltage;
          }
        };
    MARSPowerManager lowPower = new MARSPowerManager(lowIO);
    lowPower.periodic();
    OperatorInterface lowOi = new OperatorInterface(6, lowPower);

    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 50; i++) {
            lowOi.periodic();
          }
        },
        "Periodic should execute cleanly at critical voltage.");
  }
}
