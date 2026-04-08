package com.marslib.vision;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import org.junit.jupiter.api.Test;

public class MARSVisionTest {

  @Test
  public void testAmbiguityRejection() {
    // 1. Mock SwerveDrive using Mockito for method verification
    SwerveDrive mockSwerve = mock(SwerveDrive.class);

    // 2. Mock Vision IO
    AprilTagVisionIO mockVisionIO =
        new AprilTagVisionIO() {
          @Override
          public void updateInputs(AprilTagVisionIOInputs inputs) {
            inputs.estimatedPoses = new Pose3d[] {new Pose3d(), new Pose3d()};
            inputs.tagCounts = new int[] {1, 2}; // 1st frame is 1 tag, 2nd is 2 tags
            inputs.ambiguities = new double[] {0.6, 0.0}; // 1st frame is highly ambiguous (0.6)
            inputs.averageDistancesMeters = new double[] {2.0, 2.0};
            inputs.timestamps = new double[] {1.0, 2.0};
          }
        };

    MARSVision vision = new MARSVision(mockSwerve, List.of(mockVisionIO), List.of());

    // 3. Run periodic
    vision.periodic();

    // 4. Verify: addVisionMeasurement should only be called ONCE (for the frame with ambiguity 0.0
    // and tagCount 2)
    verify(mockSwerve, times(1)).addVisionMeasurement(any(), anyDouble(), any());
  }
}
