package com.marslib.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ModeConstants.*;

import com.marslib.power.MARSPowerManager;
import com.marslib.simulation.SwerveChassisPhysics;
import com.marslib.util.LoggedTunableNumber;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SwerveConstants;
import frc.robot.constants.*;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.PowerConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The core MARSLib SwerveDrive subsystem responsible for managing holonomic kinematics,
 * high-frequency odometry, dynamic PathPlanner autonomous trajectories, and active power shedding.
 *
 * <p><b>Architecture</b>
 *
 * <ul>
 *   <li><b>Odometry:</b> Powered by a background {@code PhoenixOdometryThread} that drains TalonFX
 *       Signals at 250Hz natively without waiting for the primary 50Hz RoboRIO loop. This provides
 *       unprecedented PathPlanner pathing determinism by capturing inter-tick jitter.
 *   <li><b>Physics-Linked Power Shedding:</b> Native feedback loops constantly poll the {@link
 *       MARSPowerManager}. If the RoboRIO battery hits 7.0V boundaries, Stator current limits are
 *       dynamically clamped via CANbus updates to aggressively protect from robot Brownouts during
 *       pushing matches.
 * </ul>
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
  private volatile PPHolonomicDriveController holonomicController;

  private final SwerveChassisPhysics simPhysics;
  private final com.marslib.simulation.LidarIOSim lidarSim;

  private double lastLoadShedLimit = SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT;

  /**
   * Constructs a new SwerveDrive instance.
   *
   * @param modules Array of 4 SwerveModules (FL, FR, BL, BR) wrapping hardware/simulation logic.
   * @param gyroIO The gyro IO layer (Pigeon2 or Sim) providing heading data.
   * @param powerManager The active MARSPowerManager for querying bus voltages for load shedding.
   */
  @SuppressWarnings("PMD.NullAssignment")
  public SwerveDrive(SwerveModule[] modules, GyroIO gyroIO, MARSPowerManager powerManager) {
    this.modules = java.util.Arrays.copyOf(modules, modules.length);
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

    if (frc.robot.Robot.isSimulation()) {
      simPhysics = new SwerveChassisPhysics(poseEstimator.getEstimatedPosition());
      lidarSim = new com.marslib.simulation.LidarIOSim();

      // Inject the centralized physics reference into each SwerveModuleIOSim so they
      // read wheel omegas from the single source of truth instead of local DCMotorSims.
      for (SwerveModule mod : modules) {
        mod.injectChassisPhysics(simPhysics);
      }
    } else {
      simPhysics = null;
      lidarSim = null;
    }

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
    this.cachedTransKP = transKP;
    this.cachedTransKD = transKD;
    this.cachedRotKP = rotKP;
    this.cachedRotKD = rotKD;
  }

  // Cached tunable numbers for PathPlanner PID (populated in constructor)
  private LoggedTunableNumber cachedTransKP;
  private LoggedTunableNumber cachedTransKD;
  private LoggedTunableNumber cachedRotKP;
  private LoggedTunableNumber cachedRotKD;

  // Reusable GC-free arrays for periodic loop to prevent massive RoboRIO heap churn
  private final double[] simVolts = new double[4];
  private final Rotation2d[] simAngles = new Rotation2d[4];
  private final SwerveModulePosition[] positionsForFrame = new SwerveModulePosition[4];
  private final SwerveModulePosition[] currentPositions = new SwerveModulePosition[4];

  /**
   * Configures PathPlanner's AutoBuilder for autonomous path following. Must be called exactly once
   * by {@code RobotContainer} after construction.
   *
   * <p>This is intentionally separated from the constructor to decouple the drivetrain from
   * PathPlanner, allowing unit tests to construct SwerveDrive without triggering AutoBuilder's
   * static singleton.
   */
  public void configurePathPlanner() {
    try {
      edu.wpi.first.math.system.plant.DCMotor gearbox =
          edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1)
              .withReduction(SwerveConstants.DRIVE_GEAR_RATIO);

      com.pathplanner.lib.config.ModuleConfig moduleConfig =
          new com.pathplanner.lib.config.ModuleConfig(
              SwerveConstants.WHEEL_RADIUS_METERS,
              SwerveConstants.MAX_LINEAR_SPEED_MPS,
              SwerveConstants.WHEEL_COF_STATIC,
              gearbox,
              SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT,
              1);

      RobotConfig config =
          new RobotConfig(
              SwerveConstants.ROBOT_MASS_KG,
              SwerveConstants.ROBOT_MOI_KG_M2,
              moduleConfig,
              SwerveConstants.MODULE_LOCATIONS);

      this.holonomicController =
          new PPHolonomicDriveController(
              new PIDConstants(cachedTransKP.get(), 0.0, cachedTransKD.get()),
              new PIDConstants(cachedRotKP.get(), 0.0, cachedRotKD.get()));

      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> runVelocity(speeds),
          new PathFollowingController() {
            @Override
            public ChassisSpeeds calculateRobotRelativeSpeeds(
                Pose2d currentPose, PathPlannerTrajectoryState targetState) {
              return holonomicController.calculateRobotRelativeSpeeds(currentPose, targetState);
            }

            @Override
            public void reset(Pose2d targetPose, ChassisSpeeds currentSpeeds) {
              holonomicController.reset(targetPose, currentSpeeds);
            }

            @Override
            public boolean isHolonomic() {
              return true;
            }
          },
          config,
          () -> false, // Mirroring
          this // Subsystem requirement
          );
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to configure AutoBuilder", e.getStackTrace());
      throw new RuntimeException("Failed to configure AutoBuilder", e);
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

    // Update PathPlanner PID constants if they've changed in AdvantageScope
    // Note: PathPlanner 2025 PPHolonomicDriveController is immutable.
    // Live tuning would require re-configuring AutoBuilder or using a custom controller.

    // Active Dynamic Load Shedding — only write to CAN when the limit actually changes
    double voltage = powerManager.getVoltage();
    double currentLimit;
    if (voltage < PowerConstants.NOMINAL_VOLTAGE && voltage > 0.0) {
      double slope =
          (SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT - SwerveConstants.MIN_LOAD_SHED_CURRENT)
              / (PowerConstants.NOMINAL_VOLTAGE - PowerConstants.CRITICAL_VOLTAGE);
      currentLimit =
          SwerveConstants.MIN_LOAD_SHED_CURRENT
              + (voltage - PowerConstants.CRITICAL_VOLTAGE) * slope;
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

      // Inject into generic physics boundary solver
      if (simPhysics != null) {
        simVolts[0] = modules[0].getSimDriveVoltage();
        simVolts[1] = modules[1].getSimDriveVoltage();
        simVolts[2] = modules[2].getSimDriveVoltage();
        simVolts[3] = modules[3].getSimDriveVoltage();

        simAngles[0] = modules[0].getLatestState().angle;
        simAngles[1] = modules[1].getLatestState().angle;
        simAngles[2] = modules[2].getLatestState().angle;
        simAngles[3] = modules[3].getLatestState().angle;

        simPhysics.applyModuleForces(
            simVolts, simAngles, powerManager.getVoltage(), ModeConstants.LOOP_PERIOD_SECS);

        // We override the "measured speeds" with what the physics world says is actually happening
        measuredSpeeds = simPhysics.getConstrainedSpeeds();
      }

      gyroIOSim.updateYawVelocity(
          measuredSpeeds.omegaRadiansPerSecond, ModeConstants.LOOP_PERIOD_SECS);
    }

    // Use real gyro yaw for pose estimation
    // Rotation2d yaw = ... (moved into the drain loop for high frequency accuracy)

    // Synchronous Pose Estimator Drain
    int sampleCount = modules[0].getPositionDeltas().length;
    double[] timestamps = modules[0].getOdometryTimestamps();

    for (int i = 0; i < sampleCount; i++) {
      positionsForFrame[0] = modules[0].getPositionDeltas()[i];
      positionsForFrame[1] = modules[1].getPositionDeltas()[i];
      positionsForFrame[2] = modules[2].getPositionDeltas()[i];
      positionsForFrame[3] = modules[3].getPositionDeltas()[i];

      Rotation2d frameYaw;
      if (gyroInputs.connected && gyroInputs.odometryYawPositions.length > 0) {
        frameYaw =
            new Rotation2d(
                gyroInputs
                    .odometryYawPositions[Math.min(i, gyroInputs.odometryYawPositions.length - 1)]);
      } else {
        frameYaw = new Rotation2d(gyroInputs.yawPositionRad);
      }

      double timestamp =
          (timestamps.length > i) ? timestamps[i] : edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

      poseEstimator.updateWithTime(timestamp, frameYaw, positionsForFrame);
    }

    // Log final Pose
    Pose2d currentPose = poseEstimator.getEstimatedPosition();

    // Hard-override Simulation Pose to match the 2D bounding boxes in case odometry diverges
    // heavily
    if (simPhysics != null && frc.robot.Robot.isSimulation()) {
      Pose2d simBoundedPose = simPhysics.getPose();
      currentPositions[0] = modules[0].getLatestPosition();
      currentPositions[1] = modules[1].getLatestPosition();
      currentPositions[2] = modules[2].getLatestPosition();
      currentPositions[3] = modules[3].getLatestPosition();

      poseEstimator.resetPosition(
          gyroInputs.connected
              ? new Rotation2d(gyroInputs.yawPositionRad)
              : simBoundedPose.getRotation(),
          currentPositions,
          simBoundedPose); // Force odometry matching
      currentPose = simBoundedPose;

      // Update our LiDAR point cloud based on our collision frame constraints
      if (lidarSim != null) {
        lidarSim.updateInputs(currentPose);
      }
    }

    Logger.recordOutput("SwerveDrive/Pose", currentPose);

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
    ChassisSpeeds discretizedSpeeds =
        ChassisSpeeds.discretize(speeds, ModeConstants.LOOP_PERIOD_SECS);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED_MPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }
    Logger.recordOutput("SwerveDrive/DesiredStates", states);
  }

  /**
   * Directly commands the swerve modules to the specified states. Useful for hardware testing and
   * SystemCheck diagnostics.
   *
   * @param states Array of 4 SwerveModuleStates.
   */
  public void setModuleStates(SwerveModuleState... states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED_MPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }
    Logger.recordOutput("SwerveDrive/DesiredStates", states);
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
    if (simPhysics != null) {
      simPhysics.setPose(pose);
    }
    poseEstimator.resetPosition(
        gyroInputs.connected ? new Rotation2d(gyroInputs.yawPositionRad) : pose.getRotation(),
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
