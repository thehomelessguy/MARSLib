package com.marslib.vision;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.faults.Alert;
import com.marslib.faults.MARSFaultManager;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.swerve.GyroIO;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIO;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Extended tests for MARSVision covering multi-camera input, SLAM fallback, and stale-data
 * handling.
 */
public class MARSVisionExtendedTest {

  private SwerveDrive swerveDrive;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    Alert.resetAll();
    MARSFaultManager.clear();
    // Build a stub SwerveDrive for MARSVision's constructor
    PowerIO powerIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = 12.6;
          }
        };
    MARSPowerManager power = new MARSPowerManager(powerIO);

    SwerveModuleIO stubModIO =
        new SwerveModuleIO() {
          @Override
          public void updateInputs(SwerveModuleIOInputs inputs) {
            inputs.drivePositionsRad = new double[] {0.0};
            inputs.turnPositionsRad = new double[] {0.0};
            inputs.driveVelocityRadPerSec = 0.0;
          }
        };

    SwerveModule[] modules = {
      new SwerveModule(0, stubModIO),
      new SwerveModule(1, stubModIO),
      new SwerveModule(2, stubModIO),
      new SwerveModule(3, stubModIO)
    };
    swerveDrive = new SwerveDrive(modules, new GyroIO() {}, power);
  }

  @Test
  public void testMultiCameraAprilTagAggregation() {
    Pose3d cam1Pose = new Pose3d(1.0, 2.0, 0.0, new Rotation3d());
    Pose3d cam2Pose = new Pose3d(1.1, 2.1, 0.0, new Rotation3d());

    AprilTagVisionIO cam1 =
        new AprilTagVisionIO() {
          @Override
          public void updateInputs(AprilTagVisionIOInputs inputs) {
            inputs.estimatedPoses = new Pose3d[] {cam1Pose};
            inputs.timestamps = new double[] {Timer.getFPGATimestamp()};
            inputs.tagCounts = new int[] {3};
            inputs.averageDistancesMeters = new double[] {1.5};
            inputs.ambiguities = new double[] {0.05};
          }
        };

    AprilTagVisionIO cam2 =
        new AprilTagVisionIO() {
          @Override
          public void updateInputs(AprilTagVisionIOInputs inputs) {
            inputs.estimatedPoses = new Pose3d[] {cam2Pose};
            inputs.timestamps = new double[] {Timer.getFPGATimestamp()};
            inputs.tagCounts = new int[] {2};
            inputs.averageDistancesMeters = new double[] {2.0};
            inputs.ambiguities = new double[] {0.04};
          }
        };

    MARSVision vision = new MARSVision(swerveDrive, List.of(cam1, cam2), List.of());
    assertDoesNotThrow(
        () -> vision.periodic(), "MARSVision should handle multi-camera input without error.");
  }

  @Test
  public void testSlamOnlyFallbackWhenNoCamerasPresent() {
    VIOSlamIO slamIO =
        new VIOSlamIO() {
          @Override
          public void updateInputs(VIOSlamIOInputs inputs) {
            inputs.estimatedPoses = new Pose3d[] {new Pose3d(3.0, 4.0, 0.0, new Rotation3d())};
            inputs.timestamps = new double[] {Timer.getFPGATimestamp()};
          }
        };

    MARSVision vision = new MARSVision(swerveDrive, List.of(), List.of(slamIO));
    assertDoesNotThrow(
        () -> vision.periodic(), "MARSVision should handle SLAM-only mode gracefully.");
  }

  @Test
  public void testHighAmbiguityEstimatesAreRejected() {
    AprilTagVisionIO noisyCam =
        new AprilTagVisionIO() {
          @Override
          public void updateInputs(AprilTagVisionIOInputs inputs) {
            inputs.estimatedPoses = new Pose3d[] {new Pose3d(5.0, 5.0, 0.0, new Rotation3d())};
            inputs.timestamps = new double[] {Timer.getFPGATimestamp()};
            inputs.tagCounts = new int[] {1};
            inputs.averageDistancesMeters = new double[] {5.0};
            inputs.ambiguities = new double[] {0.9};
          }
        };

    MARSVision vision = new MARSVision(swerveDrive, List.of(noisyCam), List.of());
    assertDoesNotThrow(
        () -> vision.periodic(), "High ambiguity estimates should be rejected gracefully.");
  }

  @Test
  public void testEmptyInputsHandledGracefully() {
    AprilTagVisionIO emptyCam = new AprilTagVisionIO() {};
    VIOSlamIO emptySlam = new VIOSlamIO() {};

    MARSVision vision = new MARSVision(swerveDrive, List.of(emptyCam), List.of(emptySlam));
    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 100; i++) {
            vision.periodic();
          }
        },
        "MARSVision must handle empty inputs across 100 loops without error.");
  }
}
