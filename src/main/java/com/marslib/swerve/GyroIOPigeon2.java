package com.marslib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Real hardware implementation of the gyro IO layer using a CTRE Pigeon2 IMU.
 *
 * <p>Students: The Pigeon2 is a 3-axis gyroscope connected over CAN bus. This class reads the yaw
 * (heading) at 250Hz for high-frequency odometry and the yaw velocity at 100Hz for general
 * telemetry.
 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  /**
   * Constructs a Pigeon2 gyro IO layer.
   *
   * @param canId The CAN ID of the Pigeon2.
   * @param canbus The CAN bus name (e.g. "rio" or "canivore").
   */
  public GyroIOPigeon2(int canId, String canbus) {
    pigeon = new Pigeon2(canId, canbus);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    yaw.setUpdateFrequency(250.0);
    yawVelocity.setUpdateFrequency(100.0);

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    BaseStatusSignal.refreshAll(yaw, yawVelocity);
    inputs.connected = yaw.getStatus().isOK();
    inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
