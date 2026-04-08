package com.marslib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * High-frequency CAN bus polling thread for CTRE Phoenix 6 odometry signals.
 *
 * <p>This singleton thread runs at {@link frc.robot.Constants.DriveConstants#ODOMETRY_HZ} Hz,
 * continuously sampling drive/turn encoder positions and gyro yaw from TalonFX and Pigeon2 devices.
 * Samples are buffered in thread-safe {@link BlockingQueue}s and drained by {@link
 * com.marslib.swerve.SwerveDrive#periodic()} each robot loop iteration.
 *
 * <p><b>Thread Safety:</b> All signal registration and data access is guarded by a {@link
 * ReentrantLock}. The queues use a fixed capacity of 50 to bound memory usage.
 */
public class PhoenixOdometryThread extends Thread {
  private static PhoenixOdometryThread instance = null;

  /**
   * Returns the singleton instance, creating and starting the thread on first access.
   *
   * @return The global {@link PhoenixOdometryThread} instance.
   */
  public static synchronized PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
      instance.start();
    }
    return instance;
  }

  /** Container for a batch of synchronized odometry samples from a single module. */
  public static class SyncData {
    /** Accumulated drive encoder positions (motor rotations) since the last drain. */
    public double[] drivePositions;
    /** Accumulated turn encoder positions (motor rotations) since the last drain. */
    public double[] turnPositions;
    /** FPGA timestamps corresponding to each position sample. */
    public double[] timestamps;
  }

  private final List<BaseStatusSignal> signals = new ArrayList<>();
  private final Lock signalsLock = new ReentrantLock();

  // Mapping module index to its data queue
  private final List<BlockingQueue<Double>> drivePositionQueues = new ArrayList<>();
  private final List<BlockingQueue<Double>> turnPositionQueues = new ArrayList<>();
  private final List<BlockingQueue<Double>> timestampQueues = new ArrayList<>();

  private final BlockingQueue<Double> gyroYawQueue = new ArrayBlockingQueue<>(50);
  private int gyroSignalIndex = -1;

  public PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  /**
   * Registers a swerve module's drive and turn position signals for high-frequency sampling.
   *
   * @param drivePosition The drive motor's position {@link BaseStatusSignal}.
   * @param turnPosition The turn motor's position {@link BaseStatusSignal}.
   * @return The module ID used to retrieve synchronized data via {@link #getSyncData(int)}.
   */
  public int registerModule(BaseStatusSignal drivePosition, BaseStatusSignal turnPosition) {
    signalsLock.lock();
    try {
      int id = drivePositionQueues.size();
      drivePositionQueues.add(new ArrayBlockingQueue<>(50));
      turnPositionQueues.add(new ArrayBlockingQueue<>(50));
      timestampQueues.add(new ArrayBlockingQueue<>(50));

      signals.add(drivePosition);
      signals.add(turnPosition);

      // Configure frequencies to 250Hz
      double odometryHz = frc.robot.Constants.DriveConstants.ODOMETRY_HZ;
      drivePosition.setUpdateFrequency(odometryHz);
      turnPosition.setUpdateFrequency(odometryHz);

      return id;
    } finally {
      signalsLock.unlock();
    }
  }

  /**
   * Drains all buffered odometry samples for a specific module since the last call.
   *
   * @param moduleId The module ID returned by {@link #registerModule}.
   * @return A {@link SyncData} containing synchronized drive/turn positions and timestamps.
   */
  public SyncData getSyncData(int moduleId) {
    signalsLock.lock();
    try {
      BlockingQueue<Double> dQueue = drivePositionQueues.get(moduleId);
      BlockingQueue<Double> tQueue = turnPositionQueues.get(moduleId);
      BlockingQueue<Double> tsQueue = timestampQueues.get(moduleId);

      int size = dQueue.size();
      SyncData data = new SyncData();
      data.drivePositions = new double[size];
      data.turnPositions = new double[size];
      data.timestamps = new double[size];

      for (int i = 0; i < size; i++) {
        Double driveVal = dQueue.poll();
        Double turnVal = tQueue.poll();
        Double tsVal = tsQueue.poll();
        data.drivePositions[i] = driveVal != null ? driveVal : 0.0;
        data.turnPositions[i] = turnVal != null ? turnVal : 0.0;
        data.timestamps[i] = tsVal != null ? tsVal : 0.0;
      }

      return data;
    } finally {
      signalsLock.unlock();
    }
  }

  /**
   * Registers the gyro yaw signal for high-frequency sampling alongside module signals.
   *
   * @param yawPos The Pigeon2 yaw position {@link BaseStatusSignal}.
   */
  public void registerGyro(BaseStatusSignal yawPos) {
    signalsLock.lock();
    try {
      yawPos.setUpdateFrequency(frc.robot.Constants.DriveConstants.ODOMETRY_HZ);
      signals.add(yawPos);
      gyroSignalIndex = signals.size() - 1;
    } finally {
      signalsLock.unlock();
    }
  }

  /**
   * Drains all buffered high-frequency gyro yaw samples since the last call.
   *
   * @return Array of yaw positions (rotations) accumulated since the last drain.
   */
  public double[] getGyroYawData() {
    signalsLock.lock();
    try {
      int size = gyroYawQueue.size();
      double[] data = new double[size];
      for (int i = 0; i < size; i++) {
        Double val = gyroYawQueue.poll();
        data[i] = val != null ? val : 0.0;
      }
      return data;
    } finally {
      signalsLock.unlock();
    }
  }

  @Override
  public void run() {
    while (true) {
      BaseStatusSignal[] currentSignals;
      signalsLock.lock();
      try {
        currentSignals = signals.toArray(new BaseStatusSignal[0]);
      } finally {
        signalsLock.unlock();
      }

      if (currentSignals.length == 0) {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          org.littletonrobotics.junction.Logger.recordOutput(
              "PhoenixOdometryThread/Error", e.toString());
          Thread.currentThread().interrupt();
          break;
        }
        continue;
      }

      // Wait for all signals to update, timeout after 2x the expected period
      double odometryHz = frc.robot.Constants.DriveConstants.ODOMETRY_HZ;
      BaseStatusSignal.waitForAll(2.0 / odometryHz, currentSignals);

      signalsLock.lock();
      try {
        double time = currentSignals[0].getTimestamp().getTime();

        // For each module (2 signals per module)
        for (int i = 0; i < drivePositionQueues.size(); i++) {
          // Drive signals are always added sequentially 0,1 then 2,3 etc.
          double drivePos = currentSignals[i * 2].getValueAsDouble();
          double turnPos = currentSignals[i * 2 + 1].getValueAsDouble();

          if (drivePositionQueues.get(i).remainingCapacity() > 0) {
            drivePositionQueues.get(i).offer(drivePos);
            turnPositionQueues.get(i).offer(turnPos);
            timestampQueues.get(i).offer(time);
          }
        }

        // Process gyro if registered
        if (gyroSignalIndex != -1 && gyroSignalIndex < currentSignals.length) {
          double yawPos = currentSignals[gyroSignalIndex].getValueAsDouble();
          if (gyroYawQueue.remainingCapacity() > 0) {
            gyroYawQueue.offer(yawPos);
          }
        }
      } finally {
        signalsLock.unlock();
      }
    }
  }
}
