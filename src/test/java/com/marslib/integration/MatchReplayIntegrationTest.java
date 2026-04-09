package com.marslib.integration;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import com.marslib.auto.GhostManager;
import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.RotaryMechanismIOSim;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOSim;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.MARSCowl;
import frc.robot.subsystems.MARSIntakePivot;
import frc.robot.subsystems.MARSShooter;
import frc.robot.subsystems.MARSSuperstructure;
import frc.robot.subsystems.MARSSuperstructure.SuperstructureState;
import java.io.File;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MatchReplayIntegrationTest {

  private SwerveDrive swerveDrive;
  private MARSCowl cowl;
  private MARSIntakePivot intakePivot;
  private MARSShooter floorIntake;
  private MARSShooter shooter;
  private MARSShooter feeder;
  private MARSSuperstructure superstructure;
  private MARSPowerManager powerManager;
  private GhostManager ghostManager;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    DriverStationSim.setAutonomous(true);

    powerManager = new MARSPowerManager(new PowerIOSim());
    GyroIOSim gyroSim = new GyroIOSim();
    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModule(i, new SwerveModuleIOSim(i));
    }
    swerveDrive = new SwerveDrive(modules, gyroSim, powerManager);
    swerveDrive.resetPose(new Pose2d(2, 2, new Rotation2d(0)));

    cowl = new MARSCowl(new RotaryMechanismIOSim("Cowl", 50.0, 0.5, 0.5), powerManager);
    intakePivot =
        new MARSIntakePivot(new RotaryMechanismIOSim("IntakePivot", 50.0, 0.5, 0.5), powerManager);
    floorIntake =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            powerManager);
    shooter =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            powerManager);
    feeder =
        new MARSShooter(
            new FlywheelIOSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0, 0.05),
            powerManager);

    java.util.function.Supplier<java.util.Optional<edu.wpi.first.math.geometry.Translation2d>>
        distSupplier = () -> java.util.Optional.empty();
    superstructure =
        new MARSSuperstructure(
            cowl,
            intakePivot,
            floorIntake,
            shooter,
            feeder,
            swerveDrive::getPose,
            distSupplier,
            () -> 0.0);

    ghostManager = new GhostManager();
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.cleanup();
  }

  @Test
  public void testReplayDriverInputRobustness() throws Exception {
    // 1. Create a synthetic macro file in the expected file path location
    File macroFile = new File(AutoConstants.GHOST_MACRO_FILE_PATH);
    try (PrintWriter pw =
        new PrintWriter(Files.newBufferedWriter(macroFile.toPath(), StandardCharsets.UTF_8))) {
      // time,ly,lx,rx,a,b,x,y,lb,rb,up,down,left,right
      pw.println("time,ly,lx,rx,a,b,x,y,lb,rb,up,down,left,right");

      // Idle for 0.5s
      for (double t = 0; t < 0.5; t += 0.02) {
        pw.println(
            String.format(
                "%.3f,0,0,0,false,false,false,false,false,false,false,false,false,false", t));
      }

      // Inject aggressive controller behavior for 1.0s (mashing A and B and triggers to simulate
      // erratic input)
      for (double t = 0.5; t < 1.5; t += 0.02) {
        boolean ABtn = Math.random() > 0.5;
        boolean BBtn = Math.random() > 0.5;
        pw.println(
            String.format(
                "%.3f,1.0,-1.0,0.5,%b,%b,false,false,true,false,false,false,false,false",
                t, ABtn, BBtn));
      }

      // Settle
      for (double t = 1.5; t < 2.0; t += 0.02) {
        pw.println(
            String.format(
                "%.3f,0,0,0,false,false,false,false,false,false,false,false,false,false", t));
      }
    }

    // 2. Play the synthetic file using the subsystem requirements
    Command playbackCmd = ghostManager.getPlaybackCommand();
    CommandScheduler.getInstance().schedule(playbackCmd);

    // 3. Step timeline exactly 2 seconds (100 ticks)
    for (int i = 0; i < 100; i++) {
      assertDoesNotThrow(
          () -> {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            MARSPhysicsWorld.getInstance().update(0.02);

            // Map the Ghost Manager to the Superstructure to ensure the inputs actively request
            // changes
            if (ghostManager.getA(() -> false)) {
              CommandScheduler.getInstance()
                  .schedule(superstructure.setAbsoluteState(SuperstructureState.SCORE));
            } else if (ghostManager.getB(() -> false)) {
              CommandScheduler.getInstance()
                  .schedule(superstructure.setAbsoluteState(SuperstructureState.INTAKE_RUNNING));
            } else {
              CommandScheduler.getInstance()
                  .schedule(superstructure.setAbsoluteState(SuperstructureState.STOWED));
            }
          },
          "State Machine crashed during aggressive input transition frame " + i);
    }
  }
}
