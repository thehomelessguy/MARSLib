package com.marslib.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class LoggedTunableNumber {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private DoubleSubscriber subscriber;
  private DoublePublisher publisher;
  private double lastHasChangedValue;

  public LoggedTunableNumber(String dashboardKey) {
    this.key = dashboardKey;
  }

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      this.lastHasChangedValue = defaultValue;
      var topic = NetworkTableInstance.getDefault().getTable(tableKey).getDoubleTopic(key);
      publisher = topic.publish();
      publisher.set(defaultValue);
      subscriber = topic.subscribe(defaultValue);
    }
  }

  public double get() {
    if (!hasDefault) {
      return 0.0;
    }
    if (DriverStation.isFMSAttached()) {
      return defaultValue;
    }
    return subscriber != null ? subscriber.get() : defaultValue;
  }

  public boolean hasChanged(int id) {
    if (DriverStation.isFMSAttached()) {
      return false;
    }
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }
    return false;
  }
}
