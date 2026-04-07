package com.marslib.mechanisms;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSShooterTest {

  private FlywheelIO mockIO;
  private MARSShooter shooter;

  @BeforeEach
  public void setUp() {
    mockIO = mock(FlywheelIO.class);
    shooter = new MARSShooter(mockIO);
  }

  @Test
  public void testPeriodicUpdatesInputs() {
    shooter.periodic();
    verify(mockIO).updateInputs(any(FlywheelIOInputsAutoLogged.class));
  }

  @Test
  public void testSettersPassToIO() {
    shooter.setVoltage(10.0);
    verify(mockIO).setVoltage(10.0);

    shooter.setClosedLoopVelocity(500.0, 2.5);
    verify(mockIO).setClosedLoopVelocity(500.0, 2.5);
  }

  @Test
  public void testSpinUpCommand() {
    var command = shooter.spinUpCommand();
    command.initialize();
    command.execute();

    verify(mockIO, atLeastOnce()).setClosedLoopVelocity(400.0, 0.0);
  }
}
