package com.marslib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;

/**
 * Real hardware implementation of {@link VIOSlamIO} sourcing poses from a ROS2 coprocessor.
 *
 * <p>Subscribes to a {@link Pose3d} struct topic published by a Jetson/Pi running a ROS2 SLAM node.
 * Timestamps are converted from NT4 microseconds to WPILib seconds for fusion compatibility.
 */
public class VIOSlamIOROS2 implements VIOSlamIO {

  private final StructSubscriber<Pose3d> poseSub;

  public VIOSlamIOROS2(String tableName, String topicName) {
    poseSub =
        NetworkTableInstance.getDefault()
            .getTable(tableName)
            .getStructTopic(topicName, Pose3d.struct)
            .subscribe(new Pose3d());
  }

  @Override
  public void updateInputs(VIOSlamIOInputs inputs) {
    // NT4 Struct fetching guarantees no array mismatch issues
    TimestampedObject<Pose3d>[] updates = poseSub.readQueue();

    if (updates.length > 0) {
      // Return the most recent processed update in the queue.
      TimestampedObject<Pose3d> latest = updates[updates.length - 1];

      inputs.estimatedPoses = new Pose3d[] {latest.value};
      // Convert microseconds to seconds
      inputs.timestamps = new double[] {latest.timestamp / 1_000_000.0};
    } else {
      inputs.estimatedPoses = new Pose3d[0];
      inputs.timestamps = new double[0];
    }
  }
}
