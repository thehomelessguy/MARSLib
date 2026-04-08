package com.marslib.faults;

import java.util.HashMap;
import java.util.Map;

/**
 * Central aggregator for all mission-critical error states and hardware timeouts.
 *
 * <p>Students: When a hardware layer fails (e.g. TalonFX CAN frame timeout), the exception is
 * passed here. This singleton triggers the LEDs to flash red, the Xbox Controllers to rumble, and
 * posts the Alert directly onto AdvantageScope's dashboard.
 */
public class MARSFaultManager {
  private static boolean unacknowledgedCriticalFault = false;
  private static int activeCriticalFaults = 0;

  /** Resets all fault state. Required for JUnit test isolation. */
  public static void clear() {
    unacknowledgedCriticalFault = false;
    activeCriticalFaults = 0;

    // Explicitly shut down all pending alert trackers to clear UI state
    for (Alert alert : disconnectAlerts.values()) {
      alert.set(false);
    }

    disconnectAlerts.clear();
  }

  /** Reports that a new critical fault has occurred. */
  private static void reportNewCriticalFault() {
    unacknowledgedCriticalFault = true;
  }

  /** Increments the critical fault counter and registers a dashboard flag state. */
  static void registerCriticalFault() {
    activeCriticalFaults++;
    reportNewCriticalFault();
  }

  /** Safely decrements the active fault tracker as sub-systems return online. */
  static void unregisterCriticalFault() {
    activeCriticalFaults--;
    if (activeCriticalFaults < 0) activeCriticalFaults = 0;
  }

  /**
   * Evaluates if any fault states exist in the stack currently.
   *
   * @return True if one or more hardware modules are reporting critical disconnects.
   */
  public static boolean hasActiveCriticalFaults() {
    return activeCriticalFaults > 0;
  }

  /** Returns whether a new critical fault has occurred recently. */
  public static boolean hasNewCriticalFault() {
    return unacknowledgedCriticalFault;
  }

  /**
   * Clears the new critical fault flag. Usually called by the HMI once it has acknowledged and
   * triggered the appropriate rumble/flash sequences.
   */
  public static void clearNewCriticalFault() {
    unacknowledgedCriticalFault = false;
  }

  private static final Map<String, Alert> disconnectAlerts = new HashMap<>();

  /**
   * Standard helper triggering an automatic CAN API structural error block.
   *
   * @param deviceName Name of the specific controller/device throwing the timeout (e.g.
   *     "ElevatorTalon").
   */
  public static void reportHardwareDisconnect(String deviceName) {
    if (!disconnectAlerts.containsKey(deviceName)) {
      disconnectAlerts.put(
          deviceName, new Alert("Hardware Disconnect: " + deviceName, Alert.AlertType.CRITICAL));
    }
    disconnectAlerts.get(deviceName).set(true);
  }
}
