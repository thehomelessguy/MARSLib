package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import com.marslib.power.MARSPowerManager;
import frc.robot.Constants.ElevatorConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class MARSElevatorTest {

  private LinearMechanismIO mockIO;
  private MARSPowerManager mockPowerManager;
  private MARSElevator elevator;

  @BeforeEach
  public void setUp() {
    mockIO = mock(LinearMechanismIO.class);
    mockPowerManager = mock(MARSPowerManager.class);

    elevator = new MARSElevator(mockIO, mockPowerManager);
  }

  @Test
  public void testPeriodicUpdatesInputs() {
    when(mockPowerManager.getVoltage()).thenReturn(12.0);
    elevator.periodic();

    verify(mockIO).updateInputs(any(LinearMechanismIOInputsAutoLogged.class));
  }

  @Test
  public void testLoadSheddingAtNominalVoltage() {
    when(mockPowerManager.getVoltage()).thenReturn(ElevatorConstants.NOMINAL_VOLTAGE + 1.0);

    elevator.periodic();

    verify(mockIO).setCurrentLimit(ElevatorConstants.MAX_CURRENT_AMPS);
  }

  @Test
  public void testLoadSheddingAtCriticalVoltage() {
    when(mockPowerManager.getVoltage()).thenReturn(ElevatorConstants.CRITICAL_VOLTAGE - 1.0);

    elevator.periodic();

    verify(mockIO).setCurrentLimit(ElevatorConstants.MIN_CURRENT_AMPS);
  }

  @Test
  public void testLoadSheddingInterpolatesBetweenLimits() {
    double midVoltage =
        (ElevatorConstants.NOMINAL_VOLTAGE + ElevatorConstants.CRITICAL_VOLTAGE) / 2.0;
    double expectedCurrent =
        (ElevatorConstants.MAX_CURRENT_AMPS + ElevatorConstants.MIN_CURRENT_AMPS) / 2.0;

    when(mockPowerManager.getVoltage()).thenReturn(midVoltage);

    elevator.periodic();

    ArgumentCaptor<Double> limitCaptor = ArgumentCaptor.forClass(Double.class);
    verify(mockIO).setCurrentLimit(limitCaptor.capture());

    assertEquals(expectedCurrent, limitCaptor.getValue(), 0.01, "Should proportionally shed load.");
  }

  @Test
  public void testSetTargetPositionCommandsIO() {
    doAnswer(
            invocation -> {
              LinearMechanismIOInputsAutoLogged inputs = invocation.getArgument(0);
              inputs.positionMeters = 0.5;
              inputs.targetVelocityMetersPerSec = 0.1;
              return null;
            })
        .when(mockIO)
        .updateInputs(any(LinearMechanismIOInputsAutoLogged.class));

    when(mockPowerManager.getVoltage()).thenReturn(12.0);
    elevator.periodic();

    elevator.setTargetPosition(1.0);

    verify(mockIO).setClosedLoopPosition(eq(1.0), anyDouble());
  }
}
