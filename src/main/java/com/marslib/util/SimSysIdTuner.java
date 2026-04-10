package com.marslib.util;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.ejml.simple.SimpleMatrix;

/**
 * Offline System Identification Tuner for MARSLib.
 *
 * <p>Parses AdvantageKit .wpilog telemetry files directly using WPILib's DataLogReader. Applies
 * Multiple Linear Regression (OLS) to calculate mechanistic constants (kS, kV, kA), then
 * back-calculates true Simulated Mass / Moment of Inertia for injection into dyn4j.
 */
@SuppressWarnings({
  "PMD.SystemPrintln",
  "PMD.AvoidInstantiatingObjectsInLoops",
  "PMD.DataflowAnomalyAnalysis",
  "PMD.GuardLogStatement"
})
public class SimSysIdTuner {

  private static final Logger LOGGER = Logger.getLogger(SimSysIdTuner.class.getName());

  public static class DataPoint {
    public double time;
    public double velocity;
    public double voltage;

    public DataPoint(double time, double velocity, double voltage) {
      this.time = time;
      this.velocity = velocity;
      this.voltage = voltage;
    }
  }

  /**
   * Executes regression over a given WPILog file.
   *
   * @param logFilePath Path to your AdvantageKit .wpilog file.
   * @param voltageKey AdvantageKit log key for Applied Voltage (e.g. "Shooter/AppliedVolts").
   * @param velocityKey AdvantageKit log key for Velocity (rad/s or m/s).
   * @param gearing Gear ratio (e.g., 1.5 for 1.5:1).
   * @param torqueConstant Motor Kt in N*m/A (Falcon 500 = ~0.0183).
   * @param resistance Motor internal resistance in Ohms (Falcon 500 = ~0.046).
   */
  public static void solveForSimConstants(
      String logFilePath,
      String voltageKey,
      String velocityKey,
      double gearing,
      double torqueConstant,
      double resistance) {
    LOGGER.info("==========================================");
    LOGGER.info("MARSLib Simulation Auto-Tuner Initialized");
    LOGGER.info("Target Log: " + logFilePath);
    LOGGER.info("Extracting Dynamics...");

    try {
      DataLogReader reader = new DataLogReader(logFilePath);

      int voltageEntryId = -1;
      int velocityEntryId = -1;

      Map<Long, Double> rawVolts = new HashMap<>();
      Map<Long, Double> rawVels = new HashMap<>();
      List<Long> timestamps = new ArrayList<>();

      for (DataLogRecord record : reader) {
        if (record.isStart()) {
          if (record.getStartData().name.equals(voltageKey)) {
            voltageEntryId = record.getEntry();
          } else if (record.getStartData().name.equals(velocityKey)) {
            velocityEntryId = record.getEntry();
          }
        } else if (record.getEntry() == voltageEntryId) {
          double v = record.getDouble();
          rawVolts.put(record.getTimestamp(), v);
          if (!timestamps.contains(record.getTimestamp())) timestamps.add(record.getTimestamp());
        } else if (record.getEntry() == velocityEntryId) {
          double v = record.getDouble();
          rawVels.put(record.getTimestamp(), v);
          if (!timestamps.contains(record.getTimestamp())) timestamps.add(record.getTimestamp());
        }
      }

      timestamps.sort(Long::compareTo);

      List<DataPoint> matchedData = new ArrayList<>();
      double lastVolt = 0.0;
      double lastVel = 0.0;

      for (Long t : timestamps) {
        if (rawVolts.containsKey(t)) lastVolt = rawVolts.get(t);
        if (rawVels.containsKey(t)) lastVel = rawVels.get(t);

        // WPILog timestamps are in microseconds
        matchedData.add(new DataPoint(t / 1_000_000.0, lastVel, lastVolt));
      }

      if (matchedData.size() < 10) {
        LOGGER.warning("[ERROR] Insufficient matching data found for provided keys.");
        return;
      }

      LOGGER.info("Extracted " + matchedData.size() + " synchronized telemetry frames.");
      LOGGER.info("Solving OLS Regression Matrix (Y = X * Beta)...");

      int N = matchedData.size() - 1;
      SimpleMatrix Y = new SimpleMatrix(N, 1);
      SimpleMatrix X = new SimpleMatrix(N, 3);

      for (int i = 0; i < N; i++) {
        DataPoint p1 = matchedData.get(i);
        DataPoint p2 = matchedData.get(i + 1);

        double dt = p2.time - p1.time;
        if (dt <= 0.001) dt = 0.02; // Safeguard against divide by zero (assume 20ms ds loop)

        double accel = (p2.velocity - p1.velocity) / dt;

        Y.set(i, 0, p1.voltage);
        // System Identification Model: V = kS*sign(vel) + kV*vel + kA*accel
        X.set(i, 0, Math.signum(p1.velocity));
        X.set(i, 1, p1.velocity);
        X.set(i, 2, accel);
      }

      SimpleMatrix Xt = X.transpose();
      SimpleMatrix XtX = Xt.mult(X);

      // Protect against singular matrices
      if (XtX.determinant() == 0) {
        LOGGER.warning("[ERROR] Dataset mathematically singular. Cannot perform regression.");
        return;
      }

      SimpleMatrix Beta = XtX.invert().mult(Xt).mult(Y);

      double kS = Beta.get(0, 0);
      double kV = Beta.get(1, 0);
      double kA = Beta.get(2, 0);

      // True Moment of Inertia = (kA * Gearing * TorqueConstant) / Resistance
      double calcMOI = Math.abs((kA * gearing * torqueConstant) / resistance);

      LOGGER.info("------------------------------------------");
      LOGGER.info("System Identification Constants:");
      LOGGER.info(String.format("kS (Static Friction): %.4f V", Math.abs(kS)));
      LOGGER.info(String.format("kV (Velocity):        %.4f V / (rad/s)", Math.abs(kV)));
      LOGGER.info(String.format("kA (Acceleration):    %.4f V / (rad/s^2)", Math.abs(kA)));

      LOGGER.info("------------------------------------------");
      LOGGER.info("Dyn4j / Simulation Injectable Properties:");
      LOGGER.info(String.format("True Simulated Mass / MOI: %.5f kg*m^2", calcMOI));
      LOGGER.info("==========================================");

    } catch (IOException e) {
      LOGGER.log(Level.SEVERE, "[ERROR] Failed to read WPILog file.", e);
    }
  }

  /** Offline testing entrypoint. Run natively via IDE to tune robots without deploying. */
  public static void main(String[] args) {
    // --- EXECUTION TEMPLATE ---
    // solveForSimConstants(
    //     "C:/path/to/advantagekit/Match_1.wpilog",
    //     "RealOutputs/Shooter/AppliedVolts",
    //     "RealOutputs/Shooter/VelocityRadPerSec",
    //     1.5,     // gearRatio
    //     0.0183,  // Kt (TalonFX = 0.0183)
    //     0.046    // Resistance (TalonFX = 0.046)
    // );
  }
}
