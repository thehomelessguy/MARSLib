package com.marslib.hmi;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/** Hardware IO implementation for driving WS2812B Addressable LEDs via a PWM port. */
public class LEDIOAddressable implements LEDIO {
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final int length;

  public enum State {
    DEFAULT,
    LOAD_SHEDDING,
    CRITICAL_FAULT
  }

  private State currentState = State.DEFAULT;

  public LEDIOAddressable(int pwmPort, int length) {
    this.length = length;
    leds = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

  /** Sets the LEDs to their default color state (e.g., solid blue). */
  @Override
  public void setDefaultColors() {
    currentState = State.DEFAULT;
  }

  /** Sets the LEDs to a load shedding warning color (e.g., solid orange). */
  @Override
  public void setLoadSheddingColors() {
    currentState = State.LOAD_SHEDDING;
  }

  /** Sets the LEDs to a critical fault warning color (e.g., flashing red). */
  @Override
  public void setCriticalFaultFlash() {
    currentState = State.CRITICAL_FAULT;
  }

  /** Periodically updates the underlying LED buffer based on the current state. */
  @Override
  public void update() {
    switch (currentState) {
      case CRITICAL_FAULT:
        flashRed();
        break;
      case LOAD_SHEDDING:
        solidOrange();
        break;
      case DEFAULT:
        solidBlue();
        break;
    }
    leds.setData(buffer);
  }

  private void flashRed() {
    if (((int) (Timer.getFPGATimestamp() * 10)) % 2 == 0) {
      setAll(255, 0, 0);
    } else {
      setAll(0, 0, 0);
    }
  }

  private void solidOrange() {
    setAll(255, 60, 0);
  }

  private void solidBlue() {
    setAll(0, 0, 255);
  }

  private void setAll(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }
}
