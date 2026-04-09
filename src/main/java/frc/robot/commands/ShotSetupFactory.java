package frc.robot.commands;

import com.marslib.mechanisms.*;
import com.marslib.util.ShotSetup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShotSetupFactory {
  public static ShotSetup createDefault() {
    ShotSetup shotSetup =
        new ShotSetup(
            0.1, // phaseDelay
            Math.PI / 2, // maxCowlPosition
            6000.0, // maxFlywheelSpeedRPM
            0.25, // recomputeThreshold
            5, // convergenceIters
            0.01, // convergenceEpsilon
            0.1, // minTof
            1.5, // maxTof
            new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()), // shooter at center
            new Rotation2d() // operator forward = field forward
            );

    // Distance → (RPM, CowlAngle) interpolation map
    shotSetup.addShotMapEntry(1.0, 2000.0, 0.15);
    shotSetup.addShotMapEntry(3.0, 3000.0, 0.35);
    shotSetup.addShotMapEntry(5.0, 4000.0, 0.55);
    shotSetup.addShotMapEntry(7.0, 4800.0, 0.75);
    shotSetup.addShotMapEntry(10.0, 5500.0, 1.0);

    // Distance → Time-of-Flight (seconds) interpolation map
    shotSetup.addTofMapEntry(1.0, 0.15);
    shotSetup.addTofMapEntry(3.0, 0.25);
    shotSetup.addTofMapEntry(5.0, 0.40);
    shotSetup.addTofMapEntry(7.0, 0.60);
    shotSetup.addTofMapEntry(10.0, 0.90);

    return shotSetup;
  }
}
