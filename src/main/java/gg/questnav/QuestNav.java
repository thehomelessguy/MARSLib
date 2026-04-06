package gg.questnav;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Client-side driver for the QuestNav VIO (Visual-Inertial Odometry) SLAM system. QuestNav
 * publishes its data over NetworkTables from a Meta Quest headset mounted on the robot.
 *
 * <p>Students: QuestNav provides 6-DOF pose estimation using inside-out tracking from the Quest
 * headset's built-in cameras and IMU. This data is fused with wheel odometry and AprilTag
 * localization in the {@link com.marslib.vision.MARSVision} pipeline.
 *
 * @see <a href="https://questnav.gg">QuestNav Documentation</a>
 */
public class QuestNav {
  private final DoubleArraySubscriber poseSub;
  private final DoubleSubscriber latencySub;
  private final IntegerSubscriber connectedSub;

  /** Constructs a QuestNav client reading from the default "questnav" NetworkTable. */
  public QuestNav() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("questnav");
    poseSub = table.getDoubleArrayTopic("pose").subscribe(new double[] {});
    latencySub = table.getDoubleTopic("latency").subscribe(0.0);
    connectedSub = table.getIntegerTopic("connected").subscribe(0);
  }

  /**
   * Returns whether the QuestNav headset is currently connected and publishing data.
   *
   * @return true if the headset is actively communicating.
   */
  public boolean isConnected() {
    return connectedSub.get() == 1;
  }

  /**
   * Returns the latest 6-DOF pose estimate from the Quest headset.
   *
   * <p>The pose array format is [x, y, z, roll, pitch, yaw] in meters and radians.
   *
   * @return The estimated pose in field coordinates, or an identity pose if no data is available.
   */
  public Pose3d getPose3d() {
    double[] pose = poseSub.get();
    if (pose.length >= 6) {
      return new Pose3d(
          new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
    }
    return new Pose3d();
  }

  /**
   * Returns the estimated latency of the most recent pose measurement.
   *
   * @return Latency in seconds.
   */
  public double getLatencySeconds() {
    return latencySub.get() / 1000.0; // QuestNav publishes latency in milliseconds
  }
}
