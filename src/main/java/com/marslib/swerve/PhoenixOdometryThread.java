package com.marslib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PhoenixOdometryThread extends Thread {
  private static PhoenixOdometryThread instance = null;

  public static synchronized PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
      instance.start();
    }
    return instance;
  }

  public static class SyncData {
    public double[] drivePositions;
    public double[] turnPositions;
    public double[] timestamps;
  }

  private final List<BaseStatusSignal> signals = new ArrayList<>();
  private final Lock signalsLock = new ReentrantLock();

  // Mapping module index to its data queue
  private final List<BlockingQueue<Double>> drivePositionQueues = new ArrayList<>();
  private final List<BlockingQueue<Double>> turnPositionQueues = new ArrayList<>();
  private final List<BlockingQueue<Double>> timestampQueues = new ArrayList<>();

  public PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

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
      drivePosition.setUpdateFrequency(250.0);
      turnPosition.setUpdateFrequency(250.0);

      return id;
    } finally {
      signalsLock.unlock();
    }
  }

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

  @Override
  public void run() {
    while (true) {
      BaseStatusSignal[] currentSignals;
      signalsLock.lock();
      currentSignals = signals.toArray(new BaseStatusSignal[0]);
      signalsLock.unlock();

      if (currentSignals.length == 0) {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        continue;
      }

      // Wait for all signals to update, timeout after 250hz expectations
      BaseStatusSignal.waitForAll(2.0 / 250.0, currentSignals);

      signalsLock.lock();
      try {
        double time = currentSignals[0].getTimestamp().getTime();

        // For each module (2 signals per module)
        for (int i = 0; i < drivePositionQueues.size(); i++) {
          double drivePos = currentSignals[i * 2].getValueAsDouble();
          double turnPos = currentSignals[i * 2 + 1].getValueAsDouble();

          if (drivePositionQueues.get(i).remainingCapacity() > 0) {
            drivePositionQueues.get(i).offer(drivePos);
            turnPositionQueues.get(i).offer(turnPos);
            timestampQueues.get(i).offer(time);
          }
        }
      } finally {
        signalsLock.unlock();
      }
    }
  }
}
