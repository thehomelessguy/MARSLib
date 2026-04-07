package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class SwerveModuleTest {

  private SwerveModuleIO mockIO;
  private SwerveModule module;

  @BeforeEach
  public void setUp() {
    mockIO = mock(SwerveModuleIO.class);
    module = new SwerveModule(0, mockIO);
  }

  @Test
  public void testPeriodicUpdatesInputs() {
    module.periodic();
    verify(mockIO).updateInputs(any(SwerveModuleIOInputsAutoLogged.class));
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
    // Override the mock's updateInputs to inject fake turn angle of 90 degrees
    doAnswer(
            invocation -> {
              SwerveModuleIOInputsAutoLogged inputs = invocation.getArgument(0);
              inputs.turnPositionsRad = new double[] {Math.PI / 2.0};
              return null;
            })
        .when(mockIO)
        .updateInputs(any(SwerveModuleIOInputsAutoLogged.class));

    module.periodic(); // apply mock inputs

    // Try to drive at full speed straight (0 deg).
    // Because current angle is 90 deg, error is 90. Cosine of 90 is 0.
    // Thus drive voltage should be aggressively 0 to prevent sideways drift!
    SwerveModuleState targetState =
        new SwerveModuleState(SwerveConstants.MAX_LINEAR_SPEED_MPS, new Rotation2d(0.0));

    module.setDesiredState(targetState);

    ArgumentCaptor<Double> driveVoltCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> turnVoltCaptor = ArgumentCaptor.forClass(Double.class);

    verify(mockIO).setDriveVoltage(driveVoltCaptor.capture());
    verify(mockIO).setTurnVoltage(turnVoltCaptor.capture());

    // Expect drive voltage near 0 (cos(pi/2) = 0)
    assertEquals(
        0.0,
        driveVoltCaptor.getValue(),
        0.001,
        "Drive voltage should be 0 due to cosine compensation.");

    // Target is 0, current is 90. It should turn with a negative voltage
    assertTrue(turnVoltCaptor.getValue() < 0.0, "Should apply negative voltage to turn back to 0.");
  }

  @Test
  public void testSetDesiredStateFlippingOptimization() {
    // Current angle is 0
    doAnswer(
            invocation -> {
              SwerveModuleIOInputsAutoLogged inputs = invocation.getArgument(0);
              inputs.turnPositionsRad = new double[] {0.0};
              return null;
            })
        .when(mockIO)
        .updateInputs(any(SwerveModuleIOInputsAutoLogged.class));

    module.periodic();

    // The target is 180 degrees (PI). Instead of spinning 180 degrees,
    // the module should optimize to stay at 0 but invert the drive velocity!
    SwerveModuleState targetState =
        new SwerveModuleState(SwerveConstants.MAX_LINEAR_SPEED_MPS, Rotation2d.fromDegrees(180));

    module.setDesiredState(targetState);

    ArgumentCaptor<Double> driveVoltCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> turnVoltCaptor = ArgumentCaptor.forClass(Double.class);

    verify(mockIO).setDriveVoltage(driveVoltCaptor.capture());
    verify(mockIO).setTurnVoltage(turnVoltCaptor.capture());

    // Turn voltage should be 0 because we optimize to stay at 0
    assertEquals(
        0.0, turnVoltCaptor.getValue(), 0.001, "Should not spin if optimization flips velocity.");

    // Drive voltage should be negative to go backwards towards 180
    assertTrue(driveVoltCaptor.getValue() < 0.0, "Should drive backwards instead of turning.");
  }
}
