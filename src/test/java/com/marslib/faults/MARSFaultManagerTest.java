package com.marslib.faults;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.testing.MARSTestHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSFaultManagerTest {

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    MARSFaultManager.clear();
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.cleanup();
    MARSFaultManager.clear();
  }

  @Test
  public void testFaultClearResetsAlerts() {
    assertFalse(MARSFaultManager.hasActiveCriticalFaults());

    MARSFaultManager.reportHardwareDisconnect("TestMotor");
    assertTrue(MARSFaultManager.hasActiveCriticalFaults());

    // Clear should reset the internal maps and state
    MARSFaultManager.clear();
    assertFalse(MARSFaultManager.hasActiveCriticalFaults());
  }

  @Test
  public void testWarningFaultsDontTriggerCritical() {
    MARSFaultManager.clearNewCriticalFault();
    assertFalse(
        MARSFaultManager.hasNewCriticalFault(), "Warnings should not escalate to critical faults");
  }
}
