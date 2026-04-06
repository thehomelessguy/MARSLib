package com.marslib.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;

/**
 * A tunable number that can be modified at runtime via NetworkTables for live PID tuning.
 *
 * <p>When the robot is connected to an FMS (real competition), the value is locked to the default
 * for safety and determinism. In practice/simulation mode, values can be freely adjusted from the
 * SmartDashboard or AdvantageScope.
 *
 * <p>Students: Use {@code hasChanged(int id)} to detect when a value has been modified. Each
 * consumer (identified by a unique int ID) independently tracks whether it has seen the latest
 * value.
 */
public class LoggedTunableNumber {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private DoubleSubscriber subscriber;
  private DoublePublisher publisher;

  /** Per-consumer change tracking: maps consumer ID → last seen value */
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

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

  /**
   * Returns true if the value has changed since the last time this specific consumer checked.
   *
   * @param id A unique identifier for the consumer (e.g., {@code this.hashCode()} or {@code
   *     motor.getDeviceID()}).
   * @return Whether the current value differs from the last value seen by this consumer.
   */
  public boolean hasChanged(int id) {
    if (DriverStation.isFMSAttached()) {
      return false;
    }
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }
}
