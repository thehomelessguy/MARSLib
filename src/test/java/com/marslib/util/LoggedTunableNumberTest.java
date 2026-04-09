package com.marslib.util;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.mechanisms.*;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.*;
import frc.robot.simulation.*;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Tests for LoggedTunableNumber live-tuning functionality. */
public class LoggedTunableNumberTest {

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset(); // Reset NetworkTables state for test isolation
    NetworkTableInstance.getDefault().close();
    NetworkTableInstance.getDefault();
  }

  @Test
  public void testDefaultValueIsReturned() {
    LoggedTunableNumber num = new LoggedTunableNumber("Test/DefaultValue", 3.14);
    assertEquals(3.14, num.get(), 0.001, "Should return the default value.");
  }

  @Test
  public void testGetReturnsZeroWithoutDefault() {
    LoggedTunableNumber num = new LoggedTunableNumber("Test/NoDefault");
    assertEquals(0.0, num.get(), 0.001, "Should return 0.0 if no default was set.");
  }

  @Test
  public void testHasChangedReturnsTrueOnFirstCall() {
    LoggedTunableNumber num = new LoggedTunableNumber("Test/HasChanged", 1.0);
    assertTrue(num.hasChanged(42), "First hasChanged call should always return true.");
  }

  @Test
  public void testHasChangedReturnsFalseOnSubsequentCallWithSameValue() {
    LoggedTunableNumber num = new LoggedTunableNumber("Test/NoChange", 5.0);
    num.hasChanged(1); // Prime it
    assertFalse(
        num.hasChanged(1), "Second hasChanged with same value should return false for same ID.");
  }

  @Test
  public void testMultipleConsumersTrackIndependently() {
    LoggedTunableNumber num = new LoggedTunableNumber("Test/MultiConsumer", 10.0);

    // Consumer 1 checks
    assertTrue(num.hasChanged(1), "Consumer 1 first call should be true.");

    // Consumer 2 has never checked — should also be true
    assertTrue(num.hasChanged(2), "Consumer 2 first call should also be true.");

    // Consumer 1 checks again — no change
    assertFalse(num.hasChanged(1), "Consumer 1 second call should be false (no change).");
  }

  @Test
  public void testInitDefaultOnlyOnce() {
    LoggedTunableNumber num = new LoggedTunableNumber("Test/InitOnce");
    num.initDefault(7.0);
    num.initDefault(99.0); // Should be ignored
    assertEquals(7.0, num.get(), 0.001, "initDefault should only apply the first time.");
  }
}
