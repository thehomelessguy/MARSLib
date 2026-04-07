package com.marslib.hmi;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Timer;

/**
 * LED IO layer using the CTRE CANdle (Connected via CAN bus). The CANdle supports addressable LED
 * strips and provides solid/strobe effects natively via Phoenix 6 control requests.
 *
 * <p>Students: The CANdle is a CAN-connected LED controller from CTRE. It supports up to 8 onboard
 * LEDs (indices 0-7) and an attached strip (indices 8+). This implementation uses Phoenix 6's
 * {@link SolidColor} control requests with {@link RGBWColor}.
 */
public class LEDIOCANdle implements LEDIO {
  private final CANdle candle;
  private final SolidColor solidColorRequest;

  private LEDIO.State currentState = LEDIO.State.DEFAULT;
  private LEDIO.State lastState = null;

  /**
   * Constructs a CANdle LED IO layer.
   *
   * @param canId The CAN ID of the CANdle device.
   * @param canbus The CAN bus name (e.g., "rio" or "canivore").
   * @param numLeds The total number of LEDs (onboard + strip).
   */
  public LEDIOCANdle(int canId, String canbus, int numLeds) {
    this.candle = new CANdle(canId, canbus);
    // Control all LEDs from index 0 to numLeds-1
    this.solidColorRequest = new SolidColor(0, numLeds - 1);
  }

  @Override
  public void setDefaultColors() {
    currentState = LEDIO.State.DEFAULT;
  }

  @Override
  public void setLoadSheddingColors() {
    currentState = LEDIO.State.LOAD_SHEDDING;
  }

  @Override
  public void setCriticalFaultFlash() {
    currentState = LEDIO.State.CRITICAL_FAULT;
  }

  @Override
  public void update() {
    // Only push new commands when state changes (except critical which flashes continuously)
    if (currentState == lastState && currentState != LEDIO.State.CRITICAL_FAULT) {
      return;
    }
    lastState = currentState;

    switch (currentState) {
      case CRITICAL_FAULT:
        // Flash red at ~5Hz by toggling based on FPGA timestamp
        if (((int) (Timer.getFPGATimestamp() * 10)) % 2 == 0) {
          candle.setControl(solidColorRequest.withColor(new RGBWColor(255, 0, 0)));
        } else {
          candle.setControl(solidColorRequest.withColor(new RGBWColor(0, 0, 0)));
        }
        break;
      case LOAD_SHEDDING:
        candle.setControl(solidColorRequest.withColor(new RGBWColor(255, 60, 0))); // Orange
        break;
      case DEFAULT:
        candle.setControl(solidColorRequest.withColor(new RGBWColor(0, 0, 255))); // Blue
        break;
    }
  }
}
