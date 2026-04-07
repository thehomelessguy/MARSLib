package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import java.lang.reflect.Field;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PhoenixOdometryThreadTest {

  private PhoenixOdometryThread thread;
  private BaseStatusSignal mockDriveSignal;
  private BaseStatusSignal mockTurnSignal;

  @BeforeEach
  public void setUp() {
    // Instantiate WITHOUT starting the thread to prevent infinite JNI-dependent loops
    thread = new PhoenixOdometryThread();
    mockDriveSignal = mock(BaseStatusSignal.class);
    mockTurnSignal = mock(BaseStatusSignal.class);
  }

  @Test
  public void testRegisterModuleAllocatesQueues() {
    int id1 = thread.registerModule(mockDriveSignal, mockTurnSignal);
    int id2 = thread.registerModule(mockDriveSignal, mockTurnSignal);

    assertEquals(0, id1);
    assertEquals(1, id2);

    // Verify update frequency was set
    double hz = frc.robot.Constants.DriveConstants.ODOMETRY_HZ;
    verify(mockDriveSignal, times(2)).setUpdateFrequency(hz);
    verify(mockTurnSignal, times(2)).setUpdateFrequency(hz);
  }

  @Test
  @SuppressWarnings("unchecked")
  public void testGetSyncDataDrainsQueuesFull() throws Exception {
    int id = thread.registerModule(mockDriveSignal, mockTurnSignal);

    // Use reflection to manually inject data into the BlockingQueues simulating thread execution
    Field driveQueuesField = PhoenixOdometryThread.class.getDeclaredField("drivePositionQueues");
    driveQueuesField.setAccessible(true);
    List<BlockingQueue<Double>> driveQueues =
        (List<BlockingQueue<Double>>) driveQueuesField.get(thread);

    Field turnQueuesField = PhoenixOdometryThread.class.getDeclaredField("turnPositionQueues");
    turnQueuesField.setAccessible(true);
    List<BlockingQueue<Double>> turnQueues =
        (List<BlockingQueue<Double>>) turnQueuesField.get(thread);

    Field tsQueuesField = PhoenixOdometryThread.class.getDeclaredField("timestampQueues");
    tsQueuesField.setAccessible(true);
    List<BlockingQueue<Double>> tsQueues = (List<BlockingQueue<Double>>) tsQueuesField.get(thread);

    // Inject 2 frames of data
    driveQueues.get(id).offer(1.5);
    turnQueues.get(id).offer(0.2);
    tsQueues.get(id).offer(10.0);

    driveQueues.get(id).offer(1.6);
    turnQueues.get(id).offer(0.25);
    tsQueues.get(id).offer(10.02);

    PhoenixOdometryThread.SyncData data = thread.getSyncData(id);

    assertEquals(2, data.drivePositions.length);
    assertEquals(1.5, data.drivePositions[0]);
    assertEquals(0.2, data.turnPositions[0]);
    assertEquals(10.0, data.timestamps[0]);

    assertEquals(1.6, data.drivePositions[1]);
    assertEquals(0.25, data.turnPositions[1]);
    assertEquals(10.02, data.timestamps[1]);

    // Draining again should yield empty arrays because it drains the full queue
    PhoenixOdometryThread.SyncData emptyData = thread.getSyncData(id);
    assertEquals(0, emptyData.drivePositions.length);
  }

  @Test
  @SuppressWarnings("unchecked")
  public void testGyroRegistrationAndDraining() throws Exception {
    BaseStatusSignal mockGyroSignal = mock(BaseStatusSignal.class);
    thread.registerGyro(mockGyroSignal);

    verify(mockGyroSignal).setUpdateFrequency(frc.robot.Constants.DriveConstants.ODOMETRY_HZ);

    Field gyroQueueField = PhoenixOdometryThread.class.getDeclaredField("gyroYawQueue");
    gyroQueueField.setAccessible(true);
    BlockingQueue<Double> gyroQueue = (BlockingQueue<Double>) gyroQueueField.get(thread);

    gyroQueue.offer(45.0);
    gyroQueue.offer(46.0);

    double[] data = thread.getGyroYawData();

    assertEquals(2, data.length);
    assertEquals(45.0, data[0]);
    assertEquals(46.0, data[1]);

    // Drain again
    double[] emptyData = thread.getGyroYawData();
    assertEquals(0, emptyData.length);
  }
}
