package frc.robot;

import com.marslib.auto.GhostManager;
import com.marslib.auto.MARSDiagnosticCheck;
import com.marslib.auto.ShootOnTheMoveCommand;
import com.marslib.auto.ShotSetupFactory;
import com.marslib.hmi.LEDIOAddressable;
import com.marslib.hmi.LEDManager;
import com.marslib.hmi.OperatorInterface;
import com.marslib.mechanisms.FlywheelIO;
import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.FlywheelIOTalonFX;
import com.marslib.mechanisms.LinearMechanismIO;
import com.marslib.mechanisms.LinearMechanismIOSim;
import com.marslib.mechanisms.LinearMechanismIOTalonFX;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.mechanisms.MARSShooter;
import com.marslib.mechanisms.MARSSuperstructure;
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
import com.marslib.swerve.TeleopDriveCommand;
import com.marslib.util.ShotSetup;
import com.marslib.vision.AprilTagVisionIOLimelight;
import com.marslib.vision.AprilTagVisionIOSim;
import com.marslib.vision.MARSVision;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final MARSElevator fastClimber;
  private final MARSArm cowl;
  private final MARSArm intakePivot;
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
    switch (Constants.CURRENT_MODE) {
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

          fastClimber =
              new MARSElevator(
                  new LinearMechanismIOSim(
                      "FastClimber", Constants.ClimberConstants.FAST_GEAR_RATIO, 0.05, 5.0),
                  powerManager);

          cowl =
              new MARSArm(
                  new RotaryMechanismIOSim("Cowl", Constants.CowlConstants.GEAR_RATIO, 0.5, 0.5),
                  powerManager);
          intakePivot =
              new MARSArm(
                  new RotaryMechanismIOSim(
                      "IntakePivot", Constants.IntakeConstants.PIVOT_GEAR_RATIO, 0.5, 0.5),
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
                      Constants.ShooterConstants.FEEDER_GEAR_RATIO,
                      0.025),
                  powerManager);

          ledManager =
              new LEDManager(
                  new com.marslib.hmi.LEDIOCANdle(
                      Constants.LEDConstants.CANDLE_ID,
                      Constants.LEDConstants.CANBUS,
                      Constants.LEDConstants.LENGTH),
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
                            Constants.DriveConstants.FL_DRIVE_ID,
                            Constants.DriveConstants.FL_TURN_ID,
                            Constants.DriveConstants.CANBUS)),
                    new SwerveModule(
                        1,
                        new SwerveModuleIOTalonFX(
                            Constants.DriveConstants.FR_DRIVE_ID,
                            Constants.DriveConstants.FR_TURN_ID,
                            Constants.DriveConstants.CANBUS)),
                    new SwerveModule(
                        2,
                        new SwerveModuleIOTalonFX(
                            Constants.DriveConstants.BL_DRIVE_ID,
                            Constants.DriveConstants.BL_TURN_ID,
                            Constants.DriveConstants.CANBUS)),
                    new SwerveModule(
                        3,
                        new SwerveModuleIOTalonFX(
                            Constants.DriveConstants.BR_DRIVE_ID,
                            Constants.DriveConstants.BR_TURN_ID,
                            Constants.DriveConstants.CANBUS))
                  },
                  new GyroIOPigeon2(
                      Constants.DriveConstants.PIGEON2_ID, Constants.DriveConstants.CANBUS),
                  powerManager);

          fastClimber =
              new MARSElevator(
                  new LinearMechanismIOTalonFX(
                      Constants.ClimberConstants.FAST_MOTOR_ID,
                      Constants.ClimberConstants.CANBUS,
                      Constants.ClimberConstants.FAST_GEAR_RATIO,
                      0.05,
                      false),
                  powerManager);

          cowl =
              new MARSArm(
                  new RotaryMechanismIOTalonFX(
                      Constants.CowlConstants.MOTOR_ID,
                      Constants.CowlConstants.CANBUS,
                      Constants.CowlConstants.GEAR_RATIO,
                      Constants.CowlConstants.INVERTED),
                  powerManager);
          intakePivot =
              new MARSArm(
                  new RotaryMechanismIOTalonFX(
                      Constants.IntakeConstants.PIVOT_MOTOR_ID,
                      Constants.IntakeConstants.CANBUS,
                      Constants.IntakeConstants.PIVOT_GEAR_RATIO,
                      false),
                  powerManager);

          floorIntake =
              new MARSShooter(
                  new FlywheelIOTalonFX(
                      Constants.IntakeConstants.FLOOR_MOTOR_ID,
                      Constants.IntakeConstants.CANBUS,
                      false),
                  powerManager);

          shooter =
              new MARSShooter(
                  new FlywheelIOTalonFX(
                      Constants.ShooterConstants.LM_MOTOR_ID,
                      new int[] {
                        Constants.ShooterConstants.LF_MOTOR_ID,
                        Constants.ShooterConstants.RM_MOTOR_ID,
                        Constants.ShooterConstants.RF_MOTOR_ID
                      },
                      new boolean[] {false, false, false},
                      Constants.ShooterConstants.CANBUS,
                      false),
                  powerManager);
          feeder =
              new MARSShooter(
                  new FlywheelIOTalonFX(
                      Constants.ShooterConstants.FEEDER_MOTOR_ID,
                      Constants.ShooterConstants.CANBUS,
                      false),
                  powerManager);

          ledManager =
              new LEDManager(
                  new com.marslib.hmi.LEDIOCANdle(
                      Constants.LEDConstants.CANDLE_ID,
                      Constants.LEDConstants.CANBUS,
                      Constants.LEDConstants.LENGTH),
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
          // REPLAY mode: no-op IO implementations (default interface methods) to allow
          // deterministic log replay without hardware or physics dependencies.
          powerManager = new MARSPowerManager(new PowerIOSim());

          swerveDrive =
              new SwerveDrive(
                  new SwerveModule[] {
                    new SwerveModule(0, new SwerveModuleIO() {}),
                    new SwerveModule(1, new SwerveModuleIO() {}),
                    new SwerveModule(2, new SwerveModuleIO() {}),
                    new SwerveModule(3, new SwerveModuleIO() {})
                  },
                  new GyroIO() {},
                  powerManager);

          fastClimber = new MARSElevator(new LinearMechanismIO() {}, powerManager);
          cowl = new MARSArm(new RotaryMechanismIO() {}, powerManager);
          intakePivot = new MARSArm(new RotaryMechanismIO() {}, powerManager);
          floorIntake = new MARSShooter(new FlywheelIO() {}, powerManager);
          shooter = new MARSShooter(new FlywheelIO() {}, powerManager);
          feeder = new MARSShooter(new FlywheelIO() {}, powerManager);
          ledManager =
              new LEDManager(new LEDIOAddressable(0, Constants.LEDConstants.LENGTH), powerManager);

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
            () -> {
              // Alliance-aware distance: compute distance to our alliance's hub
              Translation2d hub =
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Red
                      ? Constants.FieldConstants.RED_HUB_POS
                      : Constants.FieldConstants.BLUE_HUB_POS;
              return swerveDrive.getPose().getTranslation().getDistance(hub);
            },
            shotSetup);

    // Configure PathPlanner AutoBuilder AFTER construction — composition root owns this
    swerveDrive.configurePathPlanner();

    // Initialize the Auto Chooser
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Chooser", com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Ghost Playback", ghostManager.getPlaybackCommand());

    // 2. Configure Default Commands
    configureDefaultCommands();

    // 3. Configure Button Bindings
    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    CommandXboxController controller = operatorInterface.getController();

    // Drive with left thumbstick (translation) and right thumbstick X (rotation)
    swerveDrive.setDefaultCommand(
        new TeleopDriveCommand(
            swerveDrive,
            ghostManager,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));
  }

  private void configureButtonBindings() {
    CommandXboxController controller = operatorInterface.getController();
    CommandXboxController coPilot = new CommandXboxController(1);

    // --- DRIVE PILOT BINDINGS ---

    // Left Trigger -> Run Intake
    controller
        .leftTrigger()
        .onTrue(
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_RUNNING))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // Right Trigger -> Aim & Shoot On The Move
    controller
        .rightTrigger()
        .whileTrue(
            new ShootOnTheMoveCommand(
                swerveDrive,
                () -> {
                  double raw =
                      -Math.pow(
                              edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftY(), 0.1),
                              3.0)
                          * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
                      && edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
                          == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                },
                () -> {
                  double raw =
                      -Math.pow(
                              edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftX(), 0.1),
                              3.0)
                          * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
                      && edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
                          == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                }))
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // B Button -> Standard Stationary Shoot
    controller
        .b()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // Left Bumper -> Unjam
    controller
        .leftBumper()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.UNJAM))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // Right Bumper -> Aim and Shuttle
    controller
        .rightBumper()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // DPad Right -> Deploy Intake Without Spinning
    controller
        .povRight()
        .onTrue(
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_DOWN));

    // DPad Left -> Retract Intake (Stow)
    controller
        .povLeft()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // A Button -> Slamtake (Deploy + Run)
    controller
        .a()
        .onTrue(
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_RUNNING))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // Y Button -> Align to Climb Position (Stub)
    // X Button -> Final Climb Lineup (Stub)

    // DPad Up/Down -> Manual Climber Override
    controller
        .povUp()
        .whileTrue(
            Commands.startEnd(
                () -> fastClimber.setVoltage(12.0),
                () -> fastClimber.setVoltage(0.0),
                fastClimber));
    controller
        .povDown()
        .whileTrue(
            Commands.startEnd(
                () -> fastClimber.setVoltage(-12.0),
                () -> fastClimber.setVoltage(0.0),
                fastClimber));

    // Ghost Mode Triggers
    controller
        .back()
        .and(controller.start())
        .onTrue(
            ghostManager.registerRecordCommand(
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> controller.getRightX(),
                controller.a(),
                controller.b(),
                controller.x(),
                controller.y(),
                controller.leftBumper(),
                controller.rightBumper(),
                controller.povUp(),
                controller.povDown(),
                controller.povLeft(),
                controller.povRight()));

    // Start -> Diagnostic Hardware Check
    controller.start().onTrue(new MARSDiagnosticCheck(swerveDrive, fastClimber, cowl));

    // --- COPILOT BINDINGS ---

    // CoPilot Left Trigger -> Manual Feed
    coPilot
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  feeder.setVoltage(6.0);
                  floorIntake.setVoltage(6.0);
                },
                () -> {
                  feeder.setVoltage(0.0);
                  floorIntake.setVoltage(0.0);
                },
                feeder,
                floorIntake));

    // CoPilot Right Trigger -> Fixed Target Score (Hub)
    coPilot
        .rightTrigger()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // CoPilot Right Bumper -> Fixed Target Score (Ladder)
    coPilot
        .rightBumper()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE))
        .onFalse(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // CoPilot Left Bumper -> Cowl Home
    coPilot.leftBumper().onTrue(cowl.home());

    // CoPilot DPad Down -> Fast Climber Reverse Reverse
    coPilot
        .povDown()
        .whileTrue(
            Commands.startEnd(
                () -> fastClimber.setVoltage(-12.0),
                () -> fastClimber.setVoltage(0.0),
                fastClimber));

    // CoPilot X -> Swerve Stop
    coPilot
        .x()
        .onTrue(
            Commands.runOnce(
                () -> swerveDrive.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds()),
                swerveDrive));
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
