package com.marslib.hmi;

/**
 * Hardware abstraction interface for robot LED feedback systems.
 *
 * <p>Implementations may target WS2812B addressable strips ({@link LEDIOAddressable}) or CTRE
 * CANdle controllers. The {@link LEDManager} calls these methods each loop to reflect the robot's
 * current state (default, load shedding, or critical fault).
 */
public interface LEDIO {
  /** Sets the LED strip to its default idle color pattern (e.g., solid team color). */
  public void setDefaultColors();

  /** Sets the LED strip to a warning pattern indicating active power load shedding. */
  public void setLoadSheddingColors();

  /** Sets the LED strip to a critical fault flash pattern (e.g., flashing red). */
  public void setCriticalFaultFlash();

  /** Pushes the current color buffer to the physical LED hardware. */
  public void update();
}
