package frc.robot;

import static frc.robot.constants.ModeConstants.*;
import static frc.robot.constants.ModeConstants.Mode.*;

import com.marslib.auto.GhostManager;
import com.marslib.hmi.LEDIOAddressable;
import com.marslib.hmi.LEDManager;
import com.marslib.mechanisms.*;
import com.marslib.mechanisms.FlywheelIO;
import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.FlywheelIOTalonFX;
import com.marslib.mechanisms.LinearMechanismIO;
import com.marslib.mechanisms.LinearMechanismIOSim;
import com.marslib.mechanisms.LinearMechanismIOTalonFX;
import com.marslib.mechanisms.RotaryMechanismIO;
import com.marslib.mechanisms.RotaryMechanismIOSim;
import com.marslib.mechanisms.RotaryMechanismIOTalonFX;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOReal;
import com.marslib.power.PowerIOSim;
import com.marslib.swerve.GyroIO;
import com.marslib.swerve.GyroIOPigeon2;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIO;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.swerve.SwerveModuleIOTalonFX;
import com.marslib.util.ShotSetup;
import com.marslib.vision.AprilTagVisionIOLimelight;
import com.marslib.vision.AprilTagVisionIOSim;
import com.marslib.vision.MARSVision;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShotSetupFactory;
import frc.robot.constants.*;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.CowlConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.MARSClimber;
import frc.robot.subsystems.MARSCowl;
import frc.robot.subsystems.MARSIntakePivot;
import frc.robot.subsystems.MARSShooter;
import frc.robot.subsystems.MARSSuperstructure;
import frc.robot.subsystems.OperatorInterface;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Central Dependency Injection container for the robot. All subsystem instantiation and mode-based
 * hardware selection (REAL vs SIM vs REPLAY) happens here.
 *
 * <p>Students: This is the only file you should need to modify when changing CAN IDs, adding new
 * subsystems, or switching between hardware and simulation modes.
 */
public class RobotContainer {
  private final MARSPowerManager powerManager;
  private final OperatorInterface operatorInterface;

  private final SwerveDrive swerveDrive;
  private final MARSClimber climber;
  private final MARSCowl cowl;
  private final MARSIntakePivot intakePivot;
  private final MARSShooter floorIntake;
  private final MARSShooter shooter;
  private final MARSShooter feeder;
  private final MARSSuperstructure superstructure;
  private final GhostManager ghostManager = new GhostManager();

  @SuppressWarnings({"PMD.UnusedPrivateField", "unused"})
  private final LEDManager ledManager;

  private final MARSVision vision;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // 1. Dependency Injection based on Current Mode
    switch (ModeConstants.CURRENT_MODE) {
      case SIM:
        {
          powerManager = new MARSPowerManager(new PowerIOSim());
          GyroIOSim gyroSim = new GyroIOSim();

          swerveDrive =
              new SwerveDrive(
                  new SwerveModule[] {
                    new SwerveModule(0, new SwerveModuleIOSim(0)),
                    new SwerveModule(1, new SwerveModuleIOSim(1)),
                    new SwerveModule(2, new SwerveModuleIOSim(2)),
                    new SwerveModule(3, new SwerveModuleIOSim(3))
                  },
                  gyroSim,
                  powerManager);

          climber =
              new MARSClimber(
                  new LinearMechanismIOSim("Climber", ClimberConstants.FAST_GEAR_RATIO, 0.05, 5.0),
                  powerManager);

          cowl =
              new MARSCowl(
                  new RotaryMechanismIOSim("Cowl", CowlConstants.GEAR_RATIO, 0.5, 0.5),
                  powerManager);
          intakePivot =
              new MARSIntakePivot(
                  new RotaryMechanismIOSim(
                      "IntakePivot", IntakeConstants.PIVOT_GEAR_RATIO, 0.5, 0.5),
                  powerManager);

          floorIntake =
              new MARSShooter(
                  new FlywheelIOSim(
                      edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1.0, 0.025),
                  powerManager);
          shooter =
              new MARSShooter(
                  new FlywheelIOSim(
                      edu.wpi.first.math.system.plant.DCMotor.getFalcon500(4), 1.0, 0.05),
                  powerManager);
          feeder =
              new MARSShooter(
                  new FlywheelIOSim(
                      edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
                      ShooterConstants.FEEDER_GEAR_RATIO,
                      0.025),
                  powerManager);

          ledManager =
              new LEDManager(
                  new com.marslib.hmi.LEDIOCANdle(
                      LEDConstants.CANDLE_ID, LEDConstants.CANBUS, LEDConstants.LENGTH),
                  powerManager);

          vision =
              new MARSVision(
                  swerveDrive,
                  java.util.List.of(
                      new AprilTagVisionIOSim(
                          "limelight-front",
                          new Transform3d(
                              new Translation3d(0.3, 0.0, 0.5), new Rotation3d(0, 0, 0)),
                          () -> swerveDrive.getPose()),
                      new AprilTagVisionIOSim(
                          "limelight-back",
                          new Transform3d(
                              new Translation3d(-0.3, 0.0, 0.5), new Rotation3d(0, 0, Math.PI)),
                          () -> swerveDrive.getPose())),
                  java.util.List.of());
          break;
        }
      case REAL:
        {
          powerManager = new MARSPowerManager(new PowerIOReal());

          swerveDrive =
              new SwerveDrive(
                  new SwerveModule[] {
                    new SwerveModule(
                        0,
                        new SwerveModuleIOTalonFX(
                            DriveConstants.FL_DRIVE_ID,
                            DriveConstants.FL_TURN_ID,
                            DriveConstants.CANBUS)),
                    new SwerveModule(
                        1,
                        new SwerveModuleIOTalonFX(
                            DriveConstants.FR_DRIVE_ID,
                            DriveConstants.FR_TURN_ID,
                            DriveConstants.CANBUS)),
                    new SwerveModule(
                        2,
                        new SwerveModuleIOTalonFX(
                            DriveConstants.BL_DRIVE_ID,
                            DriveConstants.BL_TURN_ID,
                            DriveConstants.CANBUS)),
                    new SwerveModule(
                        3,
                        new SwerveModuleIOTalonFX(
                            DriveConstants.BR_DRIVE_ID,
                            DriveConstants.BR_TURN_ID,
                            DriveConstants.CANBUS))
                  },
                  new GyroIOPigeon2(DriveConstants.PIGEON2_ID, DriveConstants.CANBUS),
                  powerManager);

          climber =
              new MARSClimber(
                  new LinearMechanismIOTalonFX(
                      ClimberConstants.FAST_MOTOR_ID,
                      ClimberConstants.CANBUS,
                      ClimberConstants.FAST_GEAR_RATIO,
                      0.05,
                      false),
                  powerManager);

          cowl =
              new MARSCowl(
                  new RotaryMechanismIOTalonFX(
                      CowlConstants.MOTOR_ID,
                      CowlConstants.CANBUS,
                      CowlConstants.GEAR_RATIO,
                      CowlConstants.INVERTED),
                  powerManager);
          intakePivot =
              new MARSIntakePivot(
                  new RotaryMechanismIOTalonFX(
                      IntakeConstants.PIVOT_MOTOR_ID,
                      IntakeConstants.CANBUS,
                      IntakeConstants.PIVOT_GEAR_RATIO,
                      false),
                  powerManager);

          floorIntake =
              new MARSShooter(
                  new FlywheelIOTalonFX(
                      IntakeConstants.FLOOR_MOTOR_ID, IntakeConstants.CANBUS, false),
                  powerManager);

          shooter =
              new MARSShooter(
                  new FlywheelIOTalonFX(
                      ShooterConstants.LM_MOTOR_ID,
                      new int[] {
                        ShooterConstants.LF_MOTOR_ID,
                        ShooterConstants.RM_MOTOR_ID,
                        ShooterConstants.RF_MOTOR_ID
                      },
                      new boolean[] {false, false, false},
                      ShooterConstants.CANBUS,
                      false),
                  powerManager);
          feeder =
              new MARSShooter(
                  new FlywheelIOTalonFX(
                      ShooterConstants.FEEDER_MOTOR_ID, ShooterConstants.CANBUS, false),
                  powerManager);

          ledManager =
              new LEDManager(
                  new com.marslib.hmi.LEDIOCANdle(
                      LEDConstants.CANDLE_ID, LEDConstants.CANBUS, LEDConstants.LENGTH),
                  powerManager);

          vision =
              new MARSVision(
                  swerveDrive,
                  java.util.List.of(
                      new AprilTagVisionIOLimelight("limelight-front"),
                      new AprilTagVisionIOLimelight("limelight-back")),
                  java.util.List.of());
          break;
        }
      case REPLAY:
      default:
        {
          // REPLAY mode: no-op IO implementations to allow
          // deterministic log replay without hardware or physics dependencies.
          powerManager = new MARSPowerManager(new PowerIOSim());

          swerveDrive =
              new SwerveDrive(
                  new SwerveModule[] {
                    new SwerveModule(
                        0, com.marslib.util.ReplayIOFactory.createProxy(SwerveModuleIO.class)),
                    new SwerveModule(
                        1, com.marslib.util.ReplayIOFactory.createProxy(SwerveModuleIO.class)),
                    new SwerveModule(
                        2, com.marslib.util.ReplayIOFactory.createProxy(SwerveModuleIO.class)),
                    new SwerveModule(
                        3, com.marslib.util.ReplayIOFactory.createProxy(SwerveModuleIO.class))
                  },
                  com.marslib.util.ReplayIOFactory.createProxy(GyroIO.class),
                  powerManager);

          climber =
              new MARSClimber(
                  com.marslib.util.ReplayIOFactory.createProxy(LinearMechanismIO.class),
                  powerManager);
          cowl =
              new MARSCowl(
                  com.marslib.util.ReplayIOFactory.createProxy(RotaryMechanismIO.class),
                  powerManager);
          intakePivot =
              new MARSIntakePivot(
                  com.marslib.util.ReplayIOFactory.createProxy(RotaryMechanismIO.class),
                  powerManager);
          floorIntake =
              new MARSShooter(
                  com.marslib.util.ReplayIOFactory.createProxy(FlywheelIO.class), powerManager);
          shooter =
              new MARSShooter(
                  com.marslib.util.ReplayIOFactory.createProxy(FlywheelIO.class), powerManager);
          feeder =
              new MARSShooter(
                  com.marslib.util.ReplayIOFactory.createProxy(FlywheelIO.class), powerManager);
          ledManager = new LEDManager(new LEDIOAddressable(0, LEDConstants.LENGTH), powerManager);

          vision = new MARSVision(swerveDrive, java.util.List.of(), java.util.List.of());
          break;
        }
    }

    operatorInterface = new OperatorInterface(0, powerManager);

    // Build the SOTM shot computation utility with REBUILT shooter characterization data
    ShotSetup shotSetup = ShotSetupFactory.createDefault();

    superstructure =
        new MARSSuperstructure(
            cowl,
            intakePivot,
            floorIntake,
            shooter,
            feeder,
            swerveDrive::getPose,
            vision::getBestTargetTranslation,
            shotSetup);

    // Configure PathPlanner AutoBuilder AFTER construction — composition root owns this
    swerveDrive.configurePathPlanner();

    // Initialize the Auto Chooser
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Chooser", com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Ghost Playback", ghostManager.getPlaybackCommand());

    RobotBindings.configureBindings(
        operatorInterface,
        swerveDrive,
        ghostManager,
        superstructure,
        climber,
        cowl,
        feeder,
        floorIntake);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public MARSVision getVision() {
    return vision;
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}
