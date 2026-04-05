package com.marslib.faults;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class MARSFaultManagerTest {

  @BeforeAll
  public static void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testHardwareDisconnectTriggersFaultState() {
    // Inject hardware disconnect fault
    MARSFaultManager.reportHardwareDisconnect("TestRotaryMechanism");

    // Verify
    assertTrue(
        MARSFaultManager.hasActiveCriticalFaults(),
        "Fault Manager should have active critical faults.");
    assertTrue(
        MARSFaultManager.hasNewCriticalFault(),
        "Fault Manager should trigger the new critical fault HMI flag.");
  }
}
