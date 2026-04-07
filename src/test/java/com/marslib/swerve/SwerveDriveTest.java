package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIO;
import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveDriveTest {

  private double simulatedVoltageOverride = 12.0;
  private SwerveDrive swerveDrive;
  private SwerveModuleIOSim[] simIOs;

  @BeforeAll
  public static void setup() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  public void setUp() {
    CommandScheduler.getInstance().cancelAll();
    MARSPhysicsWorld.getInstance().resetInstance();
    simulatedVoltageOverride = 12.0;

    // Use a custom PowerIO to manipulate testing voltages independently
    PowerIO spoofedVoltageIO =
        new PowerIO() {
          @Override
          public void updateInputs(PowerIOInputs inputs) {
            inputs.voltage = simulatedVoltageOverride;
            inputs.isBrownedOut = simulatedVoltageOverride < 6.0;
          }
        };
    MARSPowerManager spoofedPowerManager = new MARSPowerManager(spoofedVoltageIO);

    GyroIOSim gyroIOSim = new GyroIOSim();
    SwerveModule[] modules = new SwerveModule[4];
    simIOs = new SwerveModuleIOSim[4];

    for (int i = 0; i < 4; i++) {
      simIOs[i] = new SwerveModuleIOSim(i);
      modules[i] = new SwerveModule(i, simIOs[i]);
    }

    swerveDrive = new SwerveDrive(modules, gyroIOSim, spoofedPowerManager);
  }

  @Test
  public void testSwerveDriveIntegrationPhysicallyMovesRobot() {
    // 1. Initial State
    swerveDrive.periodic();
    swerveDrive.simulationPeriodic();
    assertEquals(0.0, swerveDrive.getPose().getX(), 0.1);

    // 2. Command forward velocity mapping
    ChassisSpeeds targetSpeeds = new ChassisSpeeds(3.0, 0.0, 0.0);

    // 3. Fast-forward simulation by 1.0 second
    for (int i = 0; i < 50; i++) {
      DriverStationSim.notifyNewData();
      swerveDrive.runVelocity(targetSpeeds);
      CommandScheduler.getInstance().run();
      swerveDrive.periodic();
      swerveDrive.simulationPeriodic();
      MARSPhysicsWorld.getInstance().update(0.02);
      SimHooks.stepTiming(0.02);
    }

    // 4. Assert robot physically attained the intended distance purely mathematically through dyn4j
    Pose2d finalPose = swerveDrive.getPose();
    assertEquals(3.0, finalPose.getX(), 0.5, "Robot failed to traverse 3m natively through Dyn4j");
  }

  @Test
  public void testLoadSheddingRestrictsHardwareOutputs() {
    // Drop voltage to trigger protective shutdown mode
    simulatedVoltageOverride = Constants.PowerConstants.CRITICAL_VOLTAGE - 0.5;

    // Command drive
    swerveDrive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));

    // Evaluate logic frame
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().run();

    // The modules should have physically attained the strict current ceiling
    for (int i = 0; i < 4; i++) {
      assertEquals(SwerveConstants.MIN_LOAD_SHED_CURRENT, simIOs[i].getCurrentLimitAmps(), 0.1);
    }
  }
}
