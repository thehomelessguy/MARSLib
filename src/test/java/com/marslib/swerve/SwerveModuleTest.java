package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveModuleTest {

  private static class SpySwerveModuleIO implements SwerveModuleIO {
    public boolean inputsUpdated = false;
    public double injectedTurnRad = 0.0;

    public double driveVoltage = 0.0;
    public double turnVoltage = 0.0;

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
      inputsUpdated = true;
      inputs.turnPositionsRad = new double[] {injectedTurnRad};
    }

    @Override
    public void setDriveVoltage(double volts) {
      this.driveVoltage = volts;
    }

    @Override
    public void setTurnVoltage(double volts) {
      this.turnVoltage = volts;
    }
  }

  private SpySwerveModuleIO spyIO;
  private SwerveModule module;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    spyIO = new SpySwerveModuleIO();
    module = new SwerveModule(0, spyIO);
  }

  @Test
  public void testPeriodicUpdatesInputs() {
    module.periodic();
    assertTrue(spyIO.inputsUpdated);
  }

  @Test
  public void testGetPositionDeltasDefaultsToEmpty() {
    SwerveModulePosition[] deltas = module.getPositionDeltas();
    assertEquals(1, deltas.length);
    assertEquals(0.0, deltas[0].distanceMeters);
    assertEquals(0.0, deltas[0].angle.getRadians());
  }

  @Test
  public void testSetDesiredStateCosineCompensation() {
    // Current angle is 90 deg
    spyIO.injectedTurnRad = Math.PI / 2.0;

    module.periodic(); // apply mock inputs

    // Try to drive at full speed straight (0 deg).
    // Because current angle is 90 deg, error is 90. Cosine of 90 is 0.
    // Thus drive voltage should be aggressively 0 to prevent sideways drift!
    SwerveModuleState targetState =
        new SwerveModuleState(SwerveConstants.MAX_LINEAR_SPEED_MPS, new Rotation2d(0.0));

    module.setDesiredState(targetState);

    // Expect drive voltage near 0 (cos(pi/2) = 0)
    assertEquals(
        0.0, spyIO.driveVoltage, 0.001, "Drive voltage should be 0 due to cosine compensation.");

    // Target is 0, current is 90. It should turn with a negative voltage
    assertTrue(spyIO.turnVoltage < 0.0, "Should apply negative voltage to turn back to 0.");
  }

  @Test
  public void testSetDesiredStateFlippingOptimization() {
    // Current angle is 0
    spyIO.injectedTurnRad = 0.0;

    module.periodic();

    // The target is 180 degrees (PI). Instead of spinning 180 degrees,
    // the module should optimize to stay at 0 but invert the drive velocity!
    SwerveModuleState targetState =
        new SwerveModuleState(SwerveConstants.MAX_LINEAR_SPEED_MPS, Rotation2d.fromDegrees(180));

    module.setDesiredState(targetState);

    // Turn voltage should be 0 because we optimize to stay at 0
    assertEquals(0.0, spyIO.turnVoltage, 0.001, "Should not spin if optimization flips velocity.");

    // Drive voltage should be negative to go backwards towards 180
    assertTrue(spyIO.driveVoltage < 0.0, "Should drive backwards instead of turning.");
  }
}
