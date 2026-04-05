package com.marslib.hmi;

// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdleConfiguration;
// import com.ctre.phoenix.led.StrobeAnimation;

public class LEDIOCANdle implements LEDIO {
  // private final CANdle candle;
  // private final StrobeAnimation criticalAnimation;

  public enum State {
    DEFAULT,
    LOAD_SHEDDING,
    CRITICAL_FAULT
  }

  private State currentState = State.DEFAULT;
  private State lastState = null;

  public LEDIOCANdle(int canId, String canbus, int numLeds) {
    // this.candle = new CANdle(canId, canbus);

    // CANdleConfiguration config = new CANdleConfiguration();
    // config.stripType = com.ctre.phoenix.led.CANdle.LEDStripType.GRB;
    // config.brightnessScalar = 1.0;
    // candle.configAllSettings(config);

    // criticalAnimation = new StrobeAnimation(255, 0, 0, 0, 0.5, numLeds);
  }

  @Override
  public void setDefaultColors() {
    currentState = State.DEFAULT;
  }

  @Override
  public void setLoadSheddingColors() {
    currentState = State.LOAD_SHEDDING;
  }

  @Override
  public void setCriticalFaultFlash() {
    currentState = State.CRITICAL_FAULT;
  }

  @Override
  public void update() {
    if (currentState == lastState && currentState != State.CRITICAL_FAULT) {
      return;
    }
    lastState = currentState;

    // candle.clearAnimation(0);

    /*
    switch (currentState) {
      case CRITICAL_FAULT:
        candle.animate(criticalAnimation);
        break;
      case LOAD_SHEDDING:
        candle.setLEDs(255, 60, 0);
        break;
      case DEFAULT:
        candle.setLEDs(0, 0, 255);
        break;
    }
    */
  }
}
