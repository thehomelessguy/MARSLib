package com.marslib.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.marslib.power.MARSPowerManager;
import com.marslib.util.LoggedTunableNumber;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * The core MARSLib SwerveDrive subsystem responsible for managing holonomic kinematics,
 * high-frequency odometry, dynamic PathPlanner autonomous trajectories, and active power shedding.
 *
 * <p>Students: This subsystem hooks directly into AdvantageKit. All structural data is natively
 * logged to the network through the periodic loop. The Swerve modules themselves process inputs
 * through their respective hardware IO interfaces (e.g. TalonFX layers).
 */
public class SwerveDrive extends SubsystemBase {
  private final SwerveModule[] modules;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIOSim gyroIOSim; // Null if not in sim mode
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final MARSPowerManager powerManager;
  private final SysIdRoutine sysIdRoutine;
  private final Field2d field;

  private double lastLoadShedLimit = SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT;

  /**
   * Constructs a new SwerveDrive instance.
   *
   * @param modules Array of 4 SwerveModules (FL, FR, BL, BR) wrapping hardware/simulation logic.
   * @param gyroIO The gyro IO layer (Pigeon2 or Sim) providing heading data.
   * @param powerManager The active MARSPowerManager for querying bus voltages for load shedding.
   */
  public SwerveDrive(SwerveModule[] modules, GyroIO gyroIO, MARSPowerManager powerManager) {
    this.modules = modules;
    this.gyroIO = gyroIO;
    this.powerManager = powerManager;

    // Store sim reference for kinematics-derived yaw updates
    this.gyroIOSim = (gyroIO instanceof GyroIOSim) ? (GyroIOSim) gyroIO : null;

    this.kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_LOCATIONS);

    SwerveModulePosition[] initialPositions =
        new SwerveModulePosition[] {
          modules[0].getLatestPosition(), modules[1].getLatestPosition(),
          modules[2].getLatestPosition(), modules[3].getLatestPosition()
        };

    this.poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), initialPositions, new Pose2d());

    this.field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Native SysId Configuration hooked dynamically into this layer
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (edu.wpi.first.units.measure.Voltage volts) -> {
                  for (SwerveModule mod : modules) {
                    mod.setDriveVoltage(volts.in(Volts));
                    mod.setTurnVoltage(0.0);
                  }
                },
                null, // Log is handled implicitly via AdvantageKit's @AutoLog IO capturing the
                // voltages & velocities natively
                this));

    // Dynamic PID configuration for Trajectory Following
    LoggedTunableNumber transKP = new LoggedTunableNumber("Auto/Translation_kP", 5.0);
    LoggedTunableNumber transKD = new LoggedTunableNumber("Auto/Translation_kD", 0.0);
    LoggedTunableNumber rotKP = new LoggedTunableNumber("Auto/Rotation_kP", 5.0);
    LoggedTunableNumber rotKD = new LoggedTunableNumber("Auto/Rotation_kD", 0.0);

    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> runVelocity(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(transKP.get(), 0.0, transKD.get()),
              new PIDConstants(rotKP.get(), 0.0, rotKD.get())),
          config,
          () -> false, // Mirroring
          this // Subsystem requirement
          );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // Update gyro inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("SwerveDrive/Gyro", gyroInputs);

    for (SwerveModule module : modules) {
      module.periodic();
    }

    // Active Dynamic Load Shedding — only write to CAN when the limit actually changes
    double voltage = powerManager.getVoltage();
    double currentLimit;
    if (voltage < Constants.PowerConstants.NOMINAL_VOLTAGE && voltage > 0.0) {
      double slope =
          (SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT - SwerveConstants.MIN_LOAD_SHED_CURRENT)
              / (Constants.PowerConstants.NOMINAL_VOLTAGE
                  - Constants.PowerConstants.CRITICAL_VOLTAGE);
      currentLimit =
          SwerveConstants.MIN_LOAD_SHED_CURRENT
              + (voltage - Constants.PowerConstants.CRITICAL_VOLTAGE) * slope;
      currentLimit =
          Math.max(
              SwerveConstants.MIN_LOAD_SHED_CURRENT,
              Math.min(SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT, currentLimit));
    } else {
      currentLimit = SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT;
    }

    // Only push CAN writes when the limit changes by at least 1A (BUG-03 fix)
    if (Math.abs(currentLimit - lastLoadShedLimit) >= 1.0) {
      for (SwerveModule mod : modules) {
        mod.setCurrentLimit(currentLimit);
      }
      lastLoadShedLimit = currentLimit;
    }
    Logger.recordOutput("SwerveDrive/LoadShedLimitAmps", currentLimit);

    // Update simulated gyro from measured kinematics (BUG-01 fix)
    if (gyroIOSim != null) {
      ChassisSpeeds measuredSpeeds = getChassisSpeeds();
      gyroIOSim.updateYawVelocity(measuredSpeeds.omegaRadiansPerSecond, Constants.LOOP_PERIOD_SECS);
    }

    // Use real gyro yaw for pose estimation
    Rotation2d yaw =
        gyroInputs.connected ? new Rotation2d(gyroInputs.yawPositionRad) : new Rotation2d();

    // Synchronous Pose Estimator Drain
    int sampleCount = modules[0].getPositionDeltas().length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] positionsForFrame =
          new SwerveModulePosition[] {
            modules[0].getPositionDeltas()[i],
            modules[1].getPositionDeltas()[i],
            modules[2].getPositionDeltas()[i],
            modules[3].getPositionDeltas()[i]
          };

      poseEstimator.update(yaw, positionsForFrame);
    }

    // Log final Pose
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    Logger.recordOutput("SwerveDrive/Pose", currentPose);
    field.setRobotPose(currentPose);

    SwerveModuleState[] states =
        new SwerveModuleState[] {
          modules[0].getLatestState(), modules[1].getLatestState(),
          modules[2].getLatestState(), modules[3].getLatestState()
        };
    Logger.recordOutput("SwerveDrive/MeasuredStates", states);
  }

  /**
   * Drives the robot at the given velocity natively.
   *
   * <p>This is commonly triggered dynamically via PathPlanner trajectory followings or Teleop
   * joysticks. Modules are optimized for minimal rotation before applying voltages.
   *
   * @param speeds The requested translational and rotational velocities in m/s and rad/s.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_SECS);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED_MPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  /**
   * Generates a SysId Quasistatic characterization command.
   *
   * @param direction The direction of the quasistatic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Generates a SysId Dynamic characterization command.
   *
   * @param direction The direction of the dynamic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Returns the current highly-filtered pose from the SwerveDrivePoseEstimator.
   *
   * @return The current Pose2d of the robot on the field in meters.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the absolute pose of the robot. Commonly called at the start of autonomous routines.
   *
   * @param pose The new Pose2d coordinate mapped in standard WPILib field units.
   */
  public void resetPose(Pose2d pose) {
    Rotation2d yaw =
        gyroInputs.connected ? new Rotation2d(gyroInputs.yawPositionRad) : new Rotation2d();
    poseEstimator.resetPosition(
        yaw,
        new SwerveModulePosition[] {
          modules[0].getLatestPosition(), modules[1].getLatestPosition(),
          modules[2].getLatestPosition(), modules[3].getLatestPosition()
        },
        pose);
  }

  /**
   * Measures the current overall chassis speeds based on the actual measured module states.
   *
   * @return Measured ChassisSpeeds in (m/s) and (rad/s).
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        modules[0].getLatestState(), modules[1].getLatestState(),
        modules[2].getLatestState(), modules[3].getLatestState());
  }

  /**
   * Directly injects vision/SLAM data into the PoseEstimator pipeline. This is generally managed
   * natively by the MARSVision layer filtering logic.
   *
   * @param visionRobotPoseMeters The calculated field pose from the vision target (meters).
   * @param timestampSeconds The precise FPGA timestamp the image frame was captured (seconds).
   * @param visionMeasurementStdDevs The confidence matrix assigned to this measurement.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
}
