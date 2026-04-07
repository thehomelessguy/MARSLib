package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.util.ShotSetup;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSSuperstructureTest {

  private MARSArm cowl;
  private MARSArm intakePivot;
  private MARSShooter floorIntake;
  private MARSShooter shooter;
  private MARSShooter feeder;
  private MARSSuperstructure superstructure;

  private double simulatedVoltageOverride = 12.0;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    CommandScheduler.getInstance().cancelAll();
    MARSPhysicsWorld.resetInstance();

    PowerIO spoofedVoltageIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIO.PowerIOInputs inputs) {
            inputs.voltage = simulatedVoltageOverride;
            inputs.isBrownedOut = simulatedVoltageOverride < 6.0;
          }
        };

    MARSPowerManager powerManager = new MARSPowerManager(spoofedVoltageIO);

    RotaryMechanismIOSim cowlSim = new RotaryMechanismIOSim("Cowl", 50.0, 0.5, 0.5);
    cowl = new MARSArm(cowlSim, powerManager);

    RotaryMechanismIOSim intakePivotSim = new RotaryMechanismIOSim("IntakePivot", 50.0, 0.5, 0.5);
    intakePivot = new MARSArm(intakePivotSim, powerManager);

    FlywheelIOSim physicalShooterSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    shooter = new MARSShooter(physicalShooterSim);

    FlywheelIOSim physicalFloorSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    floorIntake = new MARSShooter(physicalFloorSim);

    FlywheelIOSim physicalFeederSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    feeder = new MARSShooter(physicalFeederSim);

    Supplier<Pose2d> mockSupplier = () -> new Pose2d();
    DoubleSupplier distSupplier = () -> 5.0; // Fixed dist

    ShotSetup shotSetup =
        new ShotSetup(0.0, 1.5, 6000, 0.1, 5, 0.01, 0.1, 1.0, new Transform2d(), new Rotation2d());

    superstructure =
        new MARSSuperstructure(
            cowl, intakePivot, floorIntake, shooter, feeder, mockSupplier, distSupplier, shotSetup);
  }

  @AfterEach
  public void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  public void testScoreDumpingCommand() {
    // Command SCORE phase — legal from ANY
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE).initialize();

    for (int i = 0; i < 50; i++) {
      superstructure.periodic();
      DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }
  }

  @Test
  public void testUnjamState() {
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.UNJAM).initialize();
    for (int i = 0; i < 5; i++) {
      superstructure.periodic();
    }
    assertEquals(MARSSuperstructure.SuperstructureState.UNJAM, superstructure.getCurrentState());
  }

  @Test
  public void testStateMachineTicksAndTransitionCount() {
    // Run a few ticks in STOWED
    for (int i = 0; i < 5; i++) {
      superstructure.periodic();
    }
    assertEquals(5, superstructure.getStateMachine().getTicksInCurrentState());
    assertEquals(0, superstructure.getStateMachine().getTotalTransitionCount());

    // Transition to SCORE
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE).initialize();
    superstructure.periodic();
    assertEquals(1, superstructure.getStateMachine().getTicksInCurrentState());
    assertEquals(1, superstructure.getStateMachine().getTotalTransitionCount());
  }
}
