package com.marslib.mechanisms;

import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.SuperstructureConstants;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class MARSSuperstructureTest {

  @Test
  public void testStowConstraints() {
    MARSElevator mockElevator = mock(MARSElevator.class);
    MARSArm mockArm = mock(MARSArm.class);
    MARSIntake mockIntake = mock(MARSIntake.class);
    MARSShooter mockShooter = mock(MARSShooter.class);
    Supplier<Pose2d> mockSupplier = () -> new Pose2d(); // Supplying empty pose

    // 1. Threat Condition: Elevator is completely stowed (down)
    when(mockElevator.getPositionMeters()).thenReturn(0.0);
    // Arm is physically completely retracted
    when(mockArm.getPositionRads()).thenReturn(0.0);

    MARSSuperstructure superstructure =
        new MARSSuperstructure(mockElevator, mockArm, mockIntake, mockShooter, mockSupplier);

    // 2. We command an inherently unsafe transition (Extending floor intake while elevator hasn't
    // raised itself to clear the crossbar)
    superstructure
        .setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_FLOOR)
        .initialize();

    // 3. Process machine bounds
    superstructure.periodic();

    ArgumentCaptor<Double> armGoalCaptor = ArgumentCaptor.forClass(Double.class);
    verify(mockArm, atLeastOnce()).setTargetPosition(armGoalCaptor.capture());

    // 4. Verification: The safe collision machine should realize the elevator hasn't risen enough,
    // and actively clamp the arm
    // command to prevent it extending out of the bounds.
    double safeAngleOutput = armGoalCaptor.getValue();
    org.junit.jupiter.api.Assertions.assertTrue(
        safeAngleOutput <= SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MAX_STOW,
        "Arm angle must be clamped aggressively if elevator is below safe clearance line!");
  }

  @Test
  public void testExtendedConstraints() {
    MARSElevator mockElevator = mock(MARSElevator.class);
    MARSArm mockArm = mock(MARSArm.class);
    MARSIntake mockIntake = mock(MARSIntake.class);
    MARSShooter mockShooter = mock(MARSShooter.class);
    Supplier<Pose2d> mockSupplier = () -> new Pose2d();

    // 1. Threat Condition: Arm is swinging out beyond safe zone
    when(mockArm.getPositionRads()).thenReturn(1.5);
    // Elevator is at max height safely
    when(mockElevator.getPositionMeters()).thenReturn(1.0);

    MARSSuperstructure superstructure =
        new MARSSuperstructure(mockElevator, mockArm, mockIntake, mockShooter, mockSupplier);

    // 2. Suddenly command the entire superstructure to stow immediately
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED).initialize();

    // 3. Process boundaries
    superstructure.periodic();

    ArgumentCaptor<Double> elevGoalCaptor = ArgumentCaptor.forClass(Double.class);
    verify(mockElevator, atLeastOnce()).setTargetPosition(elevGoalCaptor.capture());

    // 4. Verification: The elevator MUST NOT aggressively plummet to 0.0m while the arm is still
    // extended (which would shatter the arm)
    // It should be hard-floored at the SAFE_ELEVATOR_HEIGHT_METERS_MIN until the arm swings fully
    // backwards.
    double safeElevatorOutput = elevGoalCaptor.getValue();
    org.junit.jupiter.api.Assertions.assertTrue(
        safeElevatorOutput >= SuperstructureConstants.SAFE_ELEVATOR_HEIGHT_METERS_MIN,
        "Elevator absolutely cannot target drop below the clearance zone until the arm returns to stowed range!");
  }

  @Test
  public void testScoreDumping() {
    MARSElevator mockElevator = mock(MARSElevator.class);
    MARSArm mockArm = mock(MARSArm.class);
    MARSIntake mockIntake = mock(MARSIntake.class);
    MARSShooter mockShooter = mock(MARSShooter.class);
    Supplier<Pose2d> mockSupplier = () -> new Pose2d();

    when(mockElevator.getPositionMeters()).thenReturn(0.5);
    when(mockArm.getPositionRads()).thenReturn(1.5);

    MARSSuperstructure superstructure =
        new MARSSuperstructure(mockElevator, mockArm, mockIntake, mockShooter, mockSupplier);

    // Trigger Score phase
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE_HIGH).initialize();
    superstructure.periodic();

    // Verification: The subsystem natively commands the shooter mechanism to 400.0 rads.
    verify(mockShooter, atLeastOnce()).setClosedLoopVelocity(400.0, 0.0);

    // It should also theoretically flush `hasPiece` logic cleanly. (Though this is an internal
    // state, we can infer behavior via intake voltage)
    verify(mockIntake, atLeastOnce()).setVoltage(0.0);
  }
}
