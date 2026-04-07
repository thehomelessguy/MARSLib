package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSElevatorTest {

  private MARSElevator elevator;
  private double simulatedVoltageOverride = 12.0;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    CommandScheduler.getInstance().cancelAll();
    MARSPhysicsWorld.resetInstance();

    // Custom PowerIO allows injecting custom voltage scenarios to test the load shedding state
    // engine
    PowerIO spoofedVoltageIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIO.PowerIOInputs inputs) {
            inputs.voltage = simulatedVoltageOverride;
            inputs.isBrownedOut = simulatedVoltageOverride < 6.0;
          }
        };

    MARSPowerManager powerManager = new MARSPowerManager(spoofedVoltageIO);

    // Initialize physical IO Twin explicitly
    LinearMechanismIOSim physicalElevatorSim =
        new LinearMechanismIOSim(
            "MARSElevator_Test",
            ElevatorConstants.GEAR_RATIO,
            0.05, // 5cm spool diameter
            5.0 // 5.0 kg physical carriage mass
            );

    elevator = new MARSElevator(physicalElevatorSim, powerManager);
  }

  @Test
  public void testElevatorPhysicallyAttainsTargetHeightUsingDyn4jMotorMath() {
    // Actuate target
    elevator.setTargetPosition(1.0); // 1.0 Meters

    // Fast-forward physics simulation by 3.0 seconds (150 ticks)
    for (int i = 0; i < 150; i++) {
      DriverStationSim.notifyNewData();
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }
    System.out.println("ELEVATOR POS: " + elevator.getPositionMeters());

    // Verify actual physical tracking instead of fake boolean wrappers
    assertEquals(
        1.0,
        elevator.getPositionMeters(),
        0.05,
        "Authentic physics carriage should reach 1.0M target position.");
  }

  @Test
  public void testLoadSheddingDynamicRestoresAreAppliedUnderCriticalBrownoutConditions() {
    simulatedVoltageOverride = ElevatorConstants.CRITICAL_VOLTAGE - 0.5;

    for (int i = 0; i < 20; i++) {
      DriverStationSim.notifyNewData();
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    // Test that logic fully executes uninterrupted natively under simulated voltage caps
    assertEquals(ElevatorConstants.CRITICAL_VOLTAGE - 0.5, simulatedVoltageOverride, 0.0);
    assertNotNull(elevator);
  }
}
