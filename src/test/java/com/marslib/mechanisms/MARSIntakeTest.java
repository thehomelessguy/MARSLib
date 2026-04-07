package com.marslib.mechanisms;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSIntakeTest {

  private FlywheelIO mockIO;
  private MARSIntake intake;

  @BeforeEach
  public void setUp() {
    mockIO = mock(FlywheelIO.class);
    intake = new MARSIntake(mockIO);
  }

  @Test
  public void testPeriodicUpdatesInputs() {
    intake.periodic();
    verify(mockIO).updateInputs(any(FlywheelIOInputsAutoLogged.class));
  }

  @Test
  public void testManualControls() {
    intake.setVoltage(12.0);
    verify(mockIO).setVoltage(12.0);

    intake.stop();
    verify(mockIO).setVoltage(0.0);
  }

  @Test
  public void testCommands() {
    var runCommand = intake.runIntakeCommand();
    runCommand.initialize();
    verify(mockIO).setVoltage(12.0);
    runCommand.end(false);
    verify(mockIO).setVoltage(0.0);

    clearInvocations(mockIO);

    var reverseCommand = intake.runReverseCommand();
    reverseCommand.initialize();
    verify(mockIO).setVoltage(-12.0);
    reverseCommand.end(false);
    verify(mockIO).setVoltage(0.0);
  }
}
