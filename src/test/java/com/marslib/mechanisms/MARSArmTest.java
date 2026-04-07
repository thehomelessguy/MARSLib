package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import com.marslib.power.MARSPowerManager;
import frc.robot.Constants.ArmConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class MARSArmTest {

  private RotaryMechanismIO mockIO;
  private MARSPowerManager mockPowerManager;
  private MARSArm arm;

  @BeforeEach
  public void setUp() {
    mockIO = mock(RotaryMechanismIO.class);
    mockPowerManager = mock(MARSPowerManager.class);

    // We instantiate the subsystem. Tuning constants will initialize.
    arm = new MARSArm(mockIO, mockPowerManager);
  }

  @Test
  public void testPeriodicUpdatesInputs() {
    when(mockPowerManager.getVoltage()).thenReturn(12.0);
    arm.periodic();

    verify(mockIO).updateInputs(any(RotaryMechanismIOInputsAutoLogged.class));
  }

  @Test
  public void testLoadSheddingAtNominalVoltage() {
    // If voltage is >= NOMINAL_VOLTAGE, it should command MAX_CURRENT_AMPS
    when(mockPowerManager.getVoltage()).thenReturn(ArmConstants.NOMINAL_VOLTAGE + 1.0);

    arm.periodic();

    verify(mockIO).setCurrentLimit(ArmConstants.MAX_CURRENT_AMPS);
  }

  @Test
  public void testLoadSheddingAtCriticalVoltage() {
    // If voltage <= CRITICAL_VOLTAGE, it should command MIN_CURRENT_AMPS
    when(mockPowerManager.getVoltage()).thenReturn(ArmConstants.CRITICAL_VOLTAGE - 1.0);

    arm.periodic();

    verify(mockIO).setCurrentLimit(ArmConstants.MIN_CURRENT_AMPS);
  }

  @Test
  public void testLoadSheddingInterpolatesBetweenLimits() {
    // If voltage is exactly halfway between nominal and critical, current limit should be midway.
    double midVoltage = (ArmConstants.NOMINAL_VOLTAGE + ArmConstants.CRITICAL_VOLTAGE) / 2.0;
    double expectedCurrent = (ArmConstants.MAX_CURRENT_AMPS + ArmConstants.MIN_CURRENT_AMPS) / 2.0;

    when(mockPowerManager.getVoltage()).thenReturn(midVoltage);

    arm.periodic();

    ArgumentCaptor<Double> limitCaptor = ArgumentCaptor.forClass(Double.class);
    verify(mockIO).setCurrentLimit(limitCaptor.capture());

    assertEquals(expectedCurrent, limitCaptor.getValue(), 0.01, "Should proportionally shed load.");
  }

  @Test
  public void testSetTargetPositionCommandsIO() {
    // Verify that attempting to go to a target triggers Motion Magic IO layer wrapper
    doAnswer(
            invocation -> {
              RotaryMechanismIOInputsAutoLogged inputs = invocation.getArgument(0);
              inputs.positionRad = 1.0;
              inputs.targetVelocityRadPerSec = 0.5;
              return null;
            })
        .when(mockIO)
        .updateInputs(any(RotaryMechanismIOInputsAutoLogged.class));

    // Call periodic to read the mocked inputs
    when(mockPowerManager.getVoltage()).thenReturn(12.0);
    arm.periodic();

    arm.setTargetPosition(Math.PI);

    verify(mockIO).setClosedLoopPosition(eq(Math.PI), anyDouble());
  }
}
