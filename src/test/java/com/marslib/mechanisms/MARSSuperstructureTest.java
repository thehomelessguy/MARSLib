package com.marslib.mechanisms;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SuperstructureConstants;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MARSSuperstructureTest {

  private MARSElevator elevator;
  private MARSArm arm;
  private MARSIntake intake;
  private MARSShooter shooter;
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

    LinearMechanismIOSim physicalElevatorSim =
        new LinearMechanismIOSim("MARSElevator", ElevatorConstants.GEAR_RATIO, 0.05, 5.0);
    elevator = new MARSElevator(physicalElevatorSim, powerManager);

    RotaryMechanismIOSim physicalArmSim =
        new RotaryMechanismIOSim("MARSArm", ArmConstants.GEAR_RATIO, 0.5, 0.5);
    arm = new MARSArm(physicalArmSim, powerManager);

    FlywheelIOSim physicalIntakeSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 2.0, 0.005);
    intake = new MARSIntake(physicalIntakeSim);

    FlywheelIOSim physicalShooterSim =
        new FlywheelIOSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.002);
    shooter = new MARSShooter(physicalShooterSim);

    Supplier<Pose2d> mockSupplier = () -> new Pose2d();
    superstructure = new MARSSuperstructure(elevator, arm, intake, shooter, mockSupplier);
  }

  @Test
  public void testStowConstraintsPhysicallyEnforced() {
    // 1. Give the scheduler a few ticks to assert default stowed positions (0,0)
    for (int i = 0; i < 10; i++) {
      edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }
    assertTrue(elevator.getPositionMeters() < 0.1, "Elevator not stowed");
    assertTrue(arm.getPositionRads() < 0.1, "Arm not stowed");

    // 2. Command unsafe extension of the floor intake immediately before elevator is raised
    superstructure
        .setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_FLOOR)
        .initialize();

    for (int i = 0; i < 50; i++) {
      superstructure.periodic();
      edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    // 4. Verification: The safe collision machine should have halted the arm swing because the
    // elevator is too low!
    assertTrue(
        arm.getPositionRads() <= SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MAX_STOW + 0.1,
        "Arm swung out past the physical safe boundary when the elevator had not cleared it!");
  }

  @Test
  public void testExtendedConstraintsPhysicallyEnforced() {
    // 1. Let the superstructure command SCORE_HIGH to extend safely first.
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE_HIGH).initialize();

    // Step simulation enough for the elevator to reach target and arm to begin extending
    for (int i = 0; i < 300; i++) {
      superstructure.periodic();
      edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);
    }

    assertTrue(elevator.getPositionMeters() > 0.6, "Elevator failed to extend past safe threshold");
    // Arm will be clamped until elevator clears 0.5m; verify it's at least past the stow clamp
    assertTrue(
        arm.getPositionRads() > SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MAX_STOW,
        "Arm should have started extending once elevator cleared safe height");

    // 2. Suddenly command the entire superstructure to stow immediately
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED).initialize();

    // 3. Process boundaries briefly — the arm may retract quickly, so check mid-transition
    for (int i = 0; i < 25; i++) {
      superstructure.periodic();
      edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      MARSPhysicsWorld.getInstance().update(0.02);

      // If the arm is still extended, the elevator must stay high
      if (arm.getPositionRads() > SuperstructureConstants.SAFE_ARM_ANGLE_RAD_MIN_EXTEND) {
        assertTrue(
            elevator.getPositionMeters()
                >= SuperstructureConstants.SAFE_ELEVATOR_HEIGHT_METERS_MIN - 0.15,
            "Elevator dropped past safe threshold while arm was still extended at tick " + i);
      }
    }
  }

  @AfterEach
  public void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    // Use reflection or standard approach to unregister subsystems if necessary,
    // or just clear the scheduler completely to prevent ghost periodic calls.
    // The easiest way to stop ghosts is to literally recreate the test environment, but since
    // CommandScheduler keeps static lists, we must clear it.
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  public void testScoreDumpingCommand() {
    // Command SCORE phase
    superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE_HIGH).initialize();

    for (int i = 0; i < 50; i++) {
      superstructure.periodic();
      CommandScheduler.getInstance().run();
    }

    // Verification: The subsystem natively commands the shooter mechanism
    // Shooter should exceed 100 rads velocity within 0.5s of spin up
    assertTrue(shooter.getVelocityRadPerSec() > 100.0, "Shooter did not spin up for SCORE_HIGH");
  }
}
