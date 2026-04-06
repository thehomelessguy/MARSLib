package com.marslib.power;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware interface for power management reading. Provides inputs such as system voltage and total
 * current draw.
 */
public interface PowerIO {
  @AutoLog
  public static class PowerIOInputs {
    public double totalCurrentAmps = 0.0;
    public double voltage = 0.0;
    public double[] channelCurrentsAmps = new double[0];
  }

  public default void updateInputs(PowerIOInputs inputs) {}
}
