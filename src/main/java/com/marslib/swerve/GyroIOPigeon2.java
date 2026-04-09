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
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;

  /**
   * Constructs a Pigeon2 gyro IO layer.
   *
   * @param canId The CAN ID of the Pigeon2.
   * @param canbus The CAN bus name (e.g. "rio" or "canivore").
   */
  public GyroIOPigeon2(int canId, String canbus) {
    pigeon = new Pigeon2(canId, canbus);

    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitchVelocity = pigeon.getAngularVelocityXWorld();
    rollVelocity = pigeon.getAngularVelocityYWorld();

    yawVelocity.setUpdateFrequency(100.0);
    pitchVelocity.setUpdateFrequency(100.0);
    rollVelocity.setUpdateFrequency(100.0);
    // yaw frequency is managed by the OdometryThread

    PhoenixOdometryThread.getInstance().registerGyro(yaw);

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    BaseStatusSignal.refreshAll(yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
    inputs.connected = yaw.getStatus().isOK();
    inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble());
    inputs.pitchPositionRad = Units.degreesToRadians(pitch.getValueAsDouble());
    inputs.rollPositionRad = Units.degreesToRadians(roll.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pitchVelocity.getValueAsDouble());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelocity.getValueAsDouble());

    double[] rawOdometryYaw = PhoenixOdometryThread.getInstance().getGyroYawData();
    // In MARSLib, we'll need to store this in GyroIOInputs. Let's assume there's a field for
    // odometryYawPositions
    inputs.odometryYawPositions = new double[rawOdometryYaw.length];
    for (int i = 0; i < rawOdometryYaw.length; i++) {
      inputs.odometryYawPositions[i] = Units.degreesToRadians(rawOdometryYaw[i]);
    }
  }
}
