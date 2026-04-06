package frc.robot;

import com.marslib.auto.GhostManager;
import com.marslib.auto.MARSDiagnosticCheck;
import com.marslib.auto.SmartAssistAlign;
import com.marslib.hmi.LEDIOAddressable;
import com.marslib.hmi.LEDManager;
import com.marslib.hmi.OperatorInterface;
import com.marslib.mechanisms.LinearMechanismIOSim;
import com.marslib.mechanisms.LinearMechanismIOTalonFX;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.mechanisms.MARSSuperstructure;
import com.marslib.mechanisms.RotaryMechanismIOSim;
import com.marslib.mechanisms.RotaryMechanismIOTalonFX;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOReal;
import com.marslib.power.PowerIOSim;
import com.marslib.swerve.GyroIO;
import com.marslib.swerve.GyroIOPigeon2;
import com.marslib.swerve.GyroIOSim;
import com.marslib.swerve.SwerveConstants;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.swerve.SwerveModuleIOTalonFX;
import com.marslib.vision.MARSVision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;

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
  private final MARSSuperstructure superstructure;
  private final GhostManager ghostManager = new GhostManager();

  @SuppressWarnings("unused")
  private final LEDManager ledManager;

  @SuppressWarnings("unused")
  private final MARSVision vision;

  private final ChassisSpeeds targetSpeeds = new ChassisSpeeds();
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(6.0);
  private final PIDController headingController = new PIDController(5.0, 0, 0);
  private Rotation2d targetHeading = new Rotation2d();

  public RobotContainer() {
    // 1. Dependency Injection based on Current Mode
    switch (Constants.currentMode) {
      case SIM:
        {
          powerManager = new MARSPowerManager(new PowerIOSim());
          GyroIOSim gyroSim = new GyroIOSim();

          swerveDrive =
              new SwerveDrive(
                  new SwerveModule[] {
                    new SwerveModule(0, new SwerveModuleIOSim()),
                    new SwerveModule(1, new SwerveModuleIOSim()),
                    new SwerveModule(2, new SwerveModuleIOSim()),
                    new SwerveModule(3, new SwerveModuleIOSim())
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

          // LED uses AddressableLED in sim
          ledManager =
              new LEDManager(
                  new LEDIOAddressable(
                      Constants.LEDConstants.PWM_PORT, Constants.LEDConstants.LENGTH),
                  powerManager);
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

          // LED uses AddressableLED on real robot
          // Students: Replace with LEDIOCANdle if using a CTRE CANdle:
          //   ledManager = new LEDManager(new LEDIOCANdle(30, "rio",
          // Constants.LEDConstants.LENGTH), powerManager);
          ledManager =
              new LEDManager(
                  new LEDIOAddressable(
                      Constants.LEDConstants.PWM_PORT, Constants.LEDConstants.LENGTH),
                  powerManager);
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
                    new SwerveModule(0, new SwerveModuleIOSim()),
                    new SwerveModule(1, new SwerveModuleIOSim()),
                    new SwerveModule(2, new SwerveModuleIOSim()),
                    new SwerveModule(3, new SwerveModuleIOSim())
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
          ledManager =
              new LEDManager(
                  new LEDIOAddressable(
                      Constants.LEDConstants.PWM_PORT, Constants.LEDConstants.LENGTH),
                  powerManager);
          break;
        }
    }

    // Vision pipeline (no cameras configured by default — add your AprilTag/VIO sources here)
    // Students: To add cameras, pass lists of AprilTagVisionIO and VIOSlamIO implementations:
    //   vision = new MARSVision(swerveDrive,
    //       List.of(new AprilTagVisionIOLimelight("limelight")),
    //       List.of(new VIOSlamIOQuestNav()));
    vision = new MARSVision(swerveDrive, List.of(), List.of());

    operatorInterface = new OperatorInterface(0, powerManager);
    superstructure = new MARSSuperstructure(elevator, arm);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

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
              double linearMag = SwerveConstants.MAX_LINEAR_SPEED_MPS;
              double angularMag = SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;

              double xVal =
                  MathUtil.applyDeadband(ghostManager.getLeftY(() -> controller.getLeftY()), 0.1);
              double yVal =
                  MathUtil.applyDeadband(ghostManager.getLeftX(() -> controller.getLeftX()), 0.1);
              double omgVal =
                  MathUtil.applyDeadband(ghostManager.getRightX(() -> controller.getRightX()), 0.1);

              targetSpeeds.vxMetersPerSecond = -xLimiter.calculate(Math.pow(xVal, 3.0)) * linearMag;
              targetSpeeds.vyMetersPerSecond = -yLimiter.calculate(Math.pow(yVal, 3.0)) * linearMag;

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
                // Driver rotating actively
                targetHeading = swerveDrive.getPose().getRotation();
                targetSpeeds.omegaRadiansPerSecond =
                    -omegaLimiter.calculate(Math.pow(omgVal, 3.0)) * angularMag;
              }

              // CRITICAL: Convert teleop joystick values from Field-Relative to Robot-Centric
              ChassisSpeeds robotRelativeSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      targetSpeeds.vxMetersPerSecond,
                      targetSpeeds.vyMetersPerSecond,
                      targetSpeeds.omegaRadiansPerSecond,
                      swerveDrive.getPose().getRotation());

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

    // Left Bumper -> Smart Assist Odometry Alignment (Example Field Target)
    controller
        .leftBumper()
        .whileTrue(
            new SmartAssistAlign(
                swerveDrive,
                () ->
                    -xLimiter.calculate(
                        Math.pow(MathUtil.applyDeadband(controller.getLeftY(), 0.1), 3.0)),
                new Pose2d(5.0, 3.0, new Rotation2d(0))));
  }

  public Command getAutonomousCommand() {
    return ghostManager.getPlaybackCommand();
  }
}
