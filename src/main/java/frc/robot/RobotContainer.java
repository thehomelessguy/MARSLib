package frc.robot;

import com.marslib.auto.GhostManager;
import com.marslib.auto.MARSDiagnosticCheck;
import com.marslib.auto.ShootOnTheMoveCommand;
import com.marslib.hmi.LEDIOAddressable;
import com.marslib.hmi.LEDManager;
import com.marslib.hmi.OperatorInterface;
import com.marslib.mechanisms.FlywheelIO;
import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.FlywheelIOTalonFX;
import com.marslib.mechanisms.LinearMechanismIOSim;
import com.marslib.mechanisms.LinearMechanismIOTalonFX;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.mechanisms.MARSIntake;
import com.marslib.mechanisms.MARSShooter;
import com.marslib.mechanisms.MARSSuperstructure;
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
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.swerve.SwerveModuleIOTalonFX;
import com.marslib.vision.AprilTagVisionIOLimelight;
import com.marslib.vision.AprilTagVisionIOSim;
import com.marslib.vision.MARSVision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
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
  private final MARSElevator elevator;
  private final MARSArm arm;
  private final MARSIntake intake;
  private final MARSShooter shooter;
  private final MARSSuperstructure superstructure;
  private final GhostManager ghostManager = new GhostManager();

  private final LEDManager ledManager;

  private final MARSVision vision;

  private final ChassisSpeeds targetSpeeds = new ChassisSpeeds();
  // Fixed: Physical Acceleration Caps (15.0 m/s^2 instead of arbitrary unitless timings)
  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TELEOP_LINEAR_ACCEL_LIMIT);
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TELEOP_LINEAR_ACCEL_LIMIT);
  private final SlewRateLimiter omegaLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TELEOP_OMEGA_ACCEL_LIMIT); // Fast rotation limit
  private final PIDController headingController =
      new PIDController(Constants.DriveConstants.HEADING_KP, 0, 0);

  {
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private Rotation2d targetHeading = new Rotation2d();

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

          elevator =
              new MARSElevator(
                  new LinearMechanismIOSim(
                      "Elevator",
                      Constants.ElevatorConstants.GEAR_RATIO,
                      Constants.ElevatorConstants.SPOOL_DIAMETER_METERS,
                      Constants.ElevatorConstants.SIM_MASS_KG),
                  powerManager);
          arm =
              new MARSArm(
                  new RotaryMechanismIOSim(
                      "Arm",
                      Constants.ArmConstants.GEAR_RATIO,
                      Constants.ArmConstants.SIM_MOI,
                      Constants.ArmConstants.SIM_LENGTH),
                  powerManager);
          intake =
              new MARSIntake(
                  new FlywheelIOSim(
                      edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1.0, 0.01));
          shooter =
              new MARSShooter(
                  new FlywheelIOSim(
                      edu.wpi.first.math.system.plant.DCMotor.getFalcon500(2), 1.0, 0.05));

          // LED uses AddressableLED in sim
          ledManager =
              new LEDManager(
                  new LEDIOAddressable(
                      Constants.LEDConstants.PWM_PORT, Constants.LEDConstants.LENGTH),
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

          elevator =
              new MARSElevator(
                  new LinearMechanismIOTalonFX(
                      Constants.ElevatorConstants.MOTOR_ID,
                      Constants.ElevatorConstants.CANBUS,
                      Constants.ElevatorConstants.GEAR_RATIO,
                      Constants.ElevatorConstants.SPOOL_DIAMETER_METERS,
                      Constants.ElevatorConstants.INVERTED),
                  powerManager);
          arm =
              new MARSArm(
                  new RotaryMechanismIOTalonFX(
                      Constants.ArmConstants.MOTOR_ID,
                      Constants.ArmConstants.CANBUS,
                      Constants.ArmConstants.GEAR_RATIO,
                      Constants.ArmConstants.INVERTED),
                  powerManager);
          intake =
              new MARSIntake(
                  new FlywheelIOTalonFX(
                      Constants.IntakeConstants.MOTOR_ID, Constants.IntakeConstants.CANBUS, false));
          shooter =
              new MARSShooter(
                  new FlywheelIOTalonFX(
                      Constants.ShooterConstants.MOTOR_ID,
                      Constants.ShooterConstants.CANBUS,
                      false));

          // LED uses AddressableLED on real robot
          // Students: Replace with LEDIOCANdle if using a CTRE CANdle:
          //   ledManager = new LEDManager(new LEDIOCANdle(30, "rio",
          // Constants.LEDConstants.LENGTH), powerManager);
          ledManager =
              new LEDManager(
                  new LEDIOAddressable(
                      Constants.LEDConstants.PWM_PORT, Constants.LEDConstants.LENGTH),
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
                    new SwerveModule(0, new SwerveModuleIOSim(0)),
                    new SwerveModule(1, new SwerveModuleIOSim(1)),
                    new SwerveModule(2, new SwerveModuleIOSim(2)),
                    new SwerveModule(3, new SwerveModuleIOSim(3))
                  },
                  new GyroIO() {},
                  powerManager);

          elevator =
              new MARSElevator(
                  new LinearMechanismIOSim(
                      "Elevator",
                      Constants.ElevatorConstants.GEAR_RATIO,
                      Constants.ElevatorConstants.SPOOL_DIAMETER_METERS,
                      Constants.ElevatorConstants.SIM_MASS_KG),
                  powerManager);
          arm =
              new MARSArm(
                  new RotaryMechanismIOSim(
                      "Arm",
                      Constants.ArmConstants.GEAR_RATIO,
                      Constants.ArmConstants.SIM_MOI,
                      Constants.ArmConstants.SIM_LENGTH),
                  powerManager);
          intake = new MARSIntake(new FlywheelIO() {});
          shooter = new MARSShooter(new FlywheelIO() {});
          ledManager =
              new LEDManager(
                  new LEDIOAddressable(
                      Constants.LEDConstants.PWM_PORT, Constants.LEDConstants.LENGTH),
                  powerManager);

          vision = new MARSVision(swerveDrive, java.util.List.of(), java.util.List.of());
          break;
        }
    }

    operatorInterface = new OperatorInterface(0, powerManager);
    superstructure = new MARSSuperstructure(elevator, arm, intake, shooter, swerveDrive::getPose);

    // Configure PathPlanner AutoBuilder AFTER construction — composition root owns this
    swerveDrive.configurePathPlanner();

    headingController.enableContinuousInput(-Math.PI, Math.PI);

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
        swerveDrive.run(
            () -> {
              // Compute field-relative speeds via shared math (tested in
              // TeleopDrivePipelineTest)
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              ChassisSpeeds preSlewSpeeds =
                  com.marslib.swerve.TeleopDriveMath.computeFieldRelativeSpeeds(
                      ghostManager.getLeftY(() -> controller.getLeftY()),
                      ghostManager.getLeftX(() -> controller.getLeftX()),
                      ghostManager.getRightX(() -> controller.getRightX()),
                      isRed);

              // Post-deadband values for gyro-lock condition check
              double xVal =
                  MathUtil.applyDeadband(
                      ghostManager.getLeftY(() -> controller.getLeftY()),
                      com.marslib.swerve.TeleopDriveMath.DEADBAND);
              double yVal =
                  MathUtil.applyDeadband(
                      ghostManager.getLeftX(() -> controller.getLeftX()),
                      com.marslib.swerve.TeleopDriveMath.DEADBAND);
              double omgVal =
                  MathUtil.applyDeadband(
                      ghostManager.getRightX(() -> controller.getRightX()),
                      com.marslib.swerve.TeleopDriveMath.DEADBAND);

              // Apply slew rate limiting to translation (stateful, stays here)
              double finalX = xLimiter.calculate(preSlewSpeeds.vxMetersPerSecond);
              double finalY = yLimiter.calculate(preSlewSpeeds.vyMetersPerSecond);

              targetSpeeds.vxMetersPerSecond = finalX;
              targetSpeeds.vyMetersPerSecond = finalY;

              if (Math.abs(omgVal) <= 0.01) {
                if (Math.abs(xVal) > 0.01 || Math.abs(yVal) > 0.01) {
                  // Gyro-Lock: hold cached target heading while translating
                  targetSpeeds.omegaRadiansPerSecond =
                      headingController.calculate(
                          swerveDrive.getPose().getRotation().getRadians(),
                          targetHeading.getRadians());
                } else {
                  // Stationary, update cached heading
                  targetHeading = swerveDrive.getPose().getRotation();
                  targetSpeeds.omegaRadiansPerSecond = 0.0;
                }
              } else {
                // Driver rotating actively — use pre-slew omega from TeleopDriveMath
                targetHeading = swerveDrive.getPose().getRotation();
                targetSpeeds.omegaRadiansPerSecond =
                    omegaLimiter.calculate(preSlewSpeeds.omegaRadiansPerSecond);
              }

              // CRITICAL: Convert teleop joystick values from Field-Relative to Robot-Centric
              ChassisSpeeds robotRelativeSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      targetSpeeds.vxMetersPerSecond,
                      targetSpeeds.vyMetersPerSecond,
                      targetSpeeds.omegaRadiansPerSecond,
                      swerveDrive.getPose().getRotation());

              // Telemetry for debugging drive command pipeline
              Logger.recordOutput("Teleop/RawJoystickX", controller.getLeftY());
              Logger.recordOutput("Teleop/RawJoystickY", controller.getLeftX());
              Logger.recordOutput("Teleop/RawJoystickOmega", controller.getRightX());
              Logger.recordOutput("Teleop/PostDeadband", new double[] {xVal, yVal, omgVal});
              Logger.recordOutput(
                  "Teleop/FieldRelSpeeds",
                  new double[] {
                    targetSpeeds.vxMetersPerSecond,
                    targetSpeeds.vyMetersPerSecond,
                    targetSpeeds.omegaRadiansPerSecond
                  });
              Logger.recordOutput(
                  "Teleop/RobotRelSpeeds",
                  new double[] {
                    robotRelativeSpeeds.vxMetersPerSecond,
                    robotRelativeSpeeds.vyMetersPerSecond,
                    robotRelativeSpeeds.omegaRadiansPerSecond
                  });
              Logger.recordOutput("Teleop/GyroLockActive", Math.abs(omgVal) <= 0.01);

              swerveDrive.runVelocity(robotRelativeSpeeds);
            }));
  }

  private void configureButtonBindings() {
    CommandXboxController controller = operatorInterface.getController();

    // DPad Up -> Score High State
    controller
        .povUp()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE_HIGH));

    // DPad Down -> Stow State
    controller
        .povDown()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // DPad Right -> Intake State
    controller
        .povRight()
        .onTrue(
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_FLOOR));

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
    controller.start().onTrue(new MARSDiagnosticCheck(swerveDrive, elevator, arm, ledManager));

    // Left Bumper -> Velocity-Added Kinematic Leading (Shoot On The Move!)
    controller
        .leftBumper()
        .whileTrue(
            new ShootOnTheMoveCommand(
                swerveDrive,
                () -> {
                  double raw =
                      -Math.pow(MathUtil.applyDeadband(controller.getLeftY(), 0.1), 3.0)
                          * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                },
                () -> {
                  double raw =
                      -Math.pow(MathUtil.applyDeadband(controller.getLeftX(), 0.1), 3.0)
                          * SwerveConstants.MAX_LINEAR_SPEED_MPS;
                  if (DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red) {
                    raw = -raw;
                  }
                  return raw;
                }));
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
