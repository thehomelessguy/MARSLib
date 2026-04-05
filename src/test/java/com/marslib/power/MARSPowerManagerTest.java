package com.marslib.power;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIO;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class MARSPowerManagerTest {

  @BeforeAll
  public static void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testLoadSheddingAtLowVoltage() {
    // 1. Mock PowerIO to report 6.8V
    PowerIO mockPowerIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = 6.8;
          }
        };
    MARSPowerManager powerManager = new MARSPowerManager(mockPowerIO);

    // 2. Mock SwerveModules to monitor limits
    class MockSwerveModuleIO implements SwerveModuleIO {
      public double lastCurrentLimit = 80.0;

      @Override
      public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionsRad = new double[] {0.0};
        inputs.turnPositionsRad = new double[] {0.0};
        inputs.driveVelocityRadPerSec = 0.0;
      }

      @Override
      public void setCurrentLimit(double amps) {
        lastCurrentLimit = amps;
      }
    }

    MockSwerveModuleIO mockIO0 = new MockSwerveModuleIO();
    SwerveModule[] modules = {
      new SwerveModule(0, mockIO0),
      new SwerveModule(1, new MockSwerveModuleIO()),
      new SwerveModule(2, new MockSwerveModuleIO()),
      new SwerveModule(3, new MockSwerveModuleIO())
    };

    SwerveDrive drive = new SwerveDrive(modules, powerManager);

    // 3. Proc updates
    powerManager.periodic();
    drive.periodic();

    // 4. Assert: 6.8V is < 7V, so current limit should be clamped at the minimum which is 20.0A
    // Formula in SwerveDrive: 20.0 + (6.8 - 7.0) * 20.0 = 20.0 + (-0.2 * 20.0) = 16.0A, Clamped to
    // 20.0A
    assertEquals(
        20.0, mockIO0.lastCurrentLimit, 0.01, "SwerveDrive failed to load shed appropriately.");
  }
}
