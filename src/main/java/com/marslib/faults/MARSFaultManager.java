package com.marslib.faults;

import edu.wpi.first.wpilibj.Timer;

public class MARSFaultManager {
  private static boolean hasNewCriticalFault = false;
  private static double lastCriticalFaultTime = 0.0;
  private static int activeCriticalFaults = 0;

  /** Reports that a new critical fault has occurred. */
  private static void reportNewCriticalFault() {
    hasNewCriticalFault = true;
    lastCriticalFaultTime = Timer.getFPGATimestamp();
  }

  public static void registerCriticalFault() {
    activeCriticalFaults++;
    reportNewCriticalFault();
  }

  public static void unregisterCriticalFault() {
    activeCriticalFaults--;
    if (activeCriticalFaults < 0) activeCriticalFaults = 0;
  }

  public static boolean hasActiveCriticalFaults() {
    return activeCriticalFaults > 0;
  }

  /** Returns whether a new critical fault has occurred recently. */
  public static boolean hasNewCriticalFault() {
    return hasNewCriticalFault;
  }

  /**
   * Clears the new critical fault flag. Usually called by the HMI once it has acknowledged and
   * triggered the appropriate rumble/flash sequences.
   */
  public static void clearNewCriticalFault() {
    hasNewCriticalFault = false;
  }

  /** Helper specifically for CAN API timeouts etc. */
  public static void reportHardwareDisconnect(String deviceName) {
    new Alert("Hardware Disconnect: " + deviceName, Alert.AlertType.CRITICAL).set(true);
  }
}
