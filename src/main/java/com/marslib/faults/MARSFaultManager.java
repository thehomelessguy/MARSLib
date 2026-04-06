package com.marslib.faults;

/**
 * Central aggregator for all mission-critical error states and hardware timeouts.
 *
 * <p>Students: When a hardware layer fails (e.g. TalonFX CAN frame timeout), the exception is
 * passed here. This singleton triggers the LEDs to flash red, the Xbox Controllers to rumble, and
 * posts the Alert directly onto AdvantageScope's dashboard.
 */
public class MARSFaultManager {
  private static boolean hasNewCriticalFault = false;
  private static int activeCriticalFaults = 0;

  /** Reports that a new critical fault has occurred. */
  private static void reportNewCriticalFault() {
    hasNewCriticalFault = true;
  }

  /**
   * Increments the critical fault counter and registers a dashboard flag state. Do not call
   * directly; rely on instantiating an {@link Alert} object.
   */
  public static void registerCriticalFault() {
    activeCriticalFaults++;
    reportNewCriticalFault();
  }

  /** Safely decrements the active fault tracker as sub-systems return online. */
  public static void unregisterCriticalFault() {
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
    return hasNewCriticalFault;
  }

  /**
   * Clears the new critical fault flag. Usually called by the HMI once it has acknowledged and
   * triggered the appropriate rumble/flash sequences.
   */
  public static void clearNewCriticalFault() {
    hasNewCriticalFault = false;
  }

  /**
   * Standard helper triggering an automatic CAN API structural error block.
   *
   * @param deviceName Name of the specific controller/device throwing the timeout (e.g.
   *     "ElevatorTalon").
   */
  public static void reportHardwareDisconnect(String deviceName) {
    new Alert("Hardware Disconnect: " + deviceName, Alert.AlertType.CRITICAL).set(true);
  }
}
