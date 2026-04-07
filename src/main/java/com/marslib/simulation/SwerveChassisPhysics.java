package com.marslib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.SwerveConstants;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;

/**
 * High-fidelity 2D physics solver for a swerve chassis using the dyn4j rigid-body engine.
 *
 * <p>This class models each swerve module as an independent 1D flywheel integrated against the
 * chassis ground velocity. Motor torques are computed from WPILib's {@link DCMotor} plant model
 * (Kraken X60 FOC), and tractive forces are applied to the dyn4j {@link Body} based on a
 * Stribeck/Coulomb friction curve. The dyn4j engine then resolves wall collisions natively.
 *
 * <p>This approach provides MapleSim-tier simulation fidelity: wheel slip under heavy acceleration,
 * realistic battery sag from current draw, and physically accurate wall bounces.
 */
public class SwerveChassisPhysics {
  private static final int NUM_MODULES = SwerveConstants.MODULE_LOCATIONS.length;

  private final Body chassisBody;

  /**
   * Independent 1D flywheel integrators tracking each wheel's angular velocity (rad/s). These are
   * decoupled from the chassis ground speed to allow realistic longitudinal slip modeling.
   */
  private final double[] simulatedWheelOmegas = new double[NUM_MODULES];

  /** Simulated stator temperatures for the Kraken X60 drive motors (Celsius). */
  private final double[] statorTempCelsius = new double[] {25.0, 25.0, 25.0, 25.0};

  /**
   * Constructs the chassis physics body and registers it with the {@link MARSPhysicsWorld}.
   *
   * <p>The collision bounding rectangle uses the bumper dimensions from {@link SwerveConstants} to
   * accurately represent the robot's outer perimeter. Mass and moment of inertia are set from the
   * robot's physical constants to ensure accurate translational and rotational dynamics.
   *
   * @param startingPose The initial field-relative pose of the robot center.
   */
  public SwerveChassisPhysics(Pose2d startingPose) {
    chassisBody = new Body();
    BodyFixture fixture =
        chassisBody.addFixture(
            Geometry.createRectangle(
                SwerveConstants.BUMPER_LENGTH_METERS, SwerveConstants.BUMPER_WIDTH_METERS));

    fixture.setFriction(SwerveConstants.WALL_FRICTION);
    fixture.setRestitution(SwerveConstants.WALL_RESTITUTION);

    // Set density from mass and bumper area, then override with exact mass/MOI
    double bumperArea = SwerveConstants.BUMPER_LENGTH_METERS * SwerveConstants.BUMPER_WIDTH_METERS;
    fixture.setDensity(SwerveConstants.ROBOT_MASS_KG / bumperArea);
    chassisBody.setMass(MassType.NORMAL);

    // Override dyn4j's computed MOI with our analytically-derived block approximation
    org.dyn4j.geometry.Mass massInfo = chassisBody.getMass();
    chassisBody.setMass(
        new org.dyn4j.geometry.Mass(
            massInfo.getCenter(), SwerveConstants.ROBOT_MASS_KG, SwerveConstants.ROBOT_MOI_KG_M2));

    // Place at starting pose
    chassisBody.translate(startingPose.getX(), startingPose.getY());
    chassisBody.rotate(startingPose.getRotation().getRadians(), chassisBody.getWorldCenter());

    MARSPhysicsWorld.getInstance().registerMechanismBody("Chassis", chassisBody);
  }

  /**
   * Applies physically realistic Kraken X60 FOC motor torques and tire friction forces to the 2D
   * chassis body for a single simulation timestep.
   *
   * <p>For each module, this method:
   *
   * <ol>
   *   <li>Clamps the commanded voltage to the current battery voltage (brownout fidelity).
   *   <li>Queries the {@link DCMotor} plant model for torque and current at the wheel's angular
   *       velocity.
   *   <li>Computes the longitudinal slip velocity between the spinning wheel and the floor.
   *   <li>Selects static or kinetic friction based on the Stribeck/Coulomb slip threshold.
   *   <li>Applies tractive force to the chassis and reaction torque to the wheel flywheel.
   *   <li>Applies lateral slip friction to resist sideways drift.
   * </ol>
   *
   * @param driveVolts Commanded drive voltage for each module [0..11] (FL, FR, BL, BR).
   * @param moduleAngles Current steering angle of each module relative to the robot frame.
   * @param batteryVoltage Current simulated battery voltage from {@link MARSPhysicsWorld}. Motor
   *     commands are clamped to this value.
   * @param dtSeconds Simulation timestep duration (seconds).
   */
  public void applyModuleForces(
      double[] driveVolts, Rotation2d[] moduleAngles, double batteryVoltage, double dtSeconds) {
    DCMotor motor = DCMotor.getKrakenX60Foc(1);
    double totalCurrentAmps = 0.0;

    double[] tractiveForcesN = new double[NUM_MODULES];
    double[] lateralForcesN = new double[NUM_MODULES];
    double[] wheelRPMs = new double[NUM_MODULES];

    // Normal force per wheel assuming even weight distribution
    double normalForceN =
        (SwerveConstants.ROBOT_MASS_KG * SwerveConstants.GRAVITY_M_PER_S2) / NUM_MODULES;

    double bodyAngularVel = chassisBody.getAngularVelocity();
    Vector2 bodyLinVel = chassisBody.getLinearVelocity();
    double bodyAngle = chassisBody.getTransform().getRotationAngle();

    for (int i = 0; i < NUM_MODULES; i++) {
      Translation2d moduleOffset = SwerveConstants.MODULE_LOCATIONS[i];
      Vector2 offsetWorld = new Vector2(moduleOffset.getX(), moduleOffset.getY()).rotate(bodyAngle);

      // Ground velocity at this module's contact patch (chassis CoM velocity + rotational
      // component)
      Vector2 moduleVelWorld =
          bodyLinVel
              .copy()
              .add(new Vector2(-bodyAngularVel * offsetWorld.y, bodyAngularVel * offsetWorld.x));

      // Module heading in world frame
      Rotation2d worldModuleAngle = new Rotation2d(bodyAngle).plus(moduleAngles[i]);
      Vector2 moduleForward = new Vector2(worldModuleAngle.getCos(), worldModuleAngle.getSin());
      Vector2 moduleRight = new Vector2(-moduleForward.y, moduleForward.x);

      // Longitudinal ground speed under this wheel
      double groundSpeedMps = moduleVelWorld.dot(moduleForward);

      // --- 1. MOTOR DYNAMICS (1D Flywheel) ---
      // Clamp commanded voltage to available battery voltage (brownout fidelity)
      double safeBatteryVoltage = Math.max(batteryVoltage, 0.01);
      double clampedVoltage =
          Math.copySign(Math.min(Math.abs(driveVolts[i]), safeBatteryVoltage), driveVolts[i]);

      double motorRadPerSec = simulatedWheelOmegas[i] * SwerveConstants.DRIVE_GEAR_RATIO;
      double currentDrawAmps = motor.getCurrent(motorRadPerSec, clampedVoltage);

      // Enforce stator current limit just like real TalonFX firmware.
      currentDrawAmps =
          Math.copySign(
              Math.min(Math.abs(currentDrawAmps), SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT),
              currentDrawAmps);

      // Compute the actual motor terminal voltage after current limiting.
      // From the DC motor equation: I = (V - ω/Kv) / R  →  V = I·R + ω/Kv
      double effectiveVoltage =
          currentDrawAmps * motor.rOhms + motorRadPerSec / motor.KvRadPerSecPerVolt;

      // The motor driver cannot produce more than battery voltage.
      if (Math.abs(effectiveVoltage) > safeBatteryVoltage) {
        effectiveVoltage = Math.copySign(safeBatteryVoltage, effectiveVoltage);
        // Recalculate true current at voltage limit
        currentDrawAmps =
            (effectiveVoltage - motorRadPerSec / motor.KvRadPerSecPerVolt) / motor.rOhms;
      }

      // Electrical Power = V_effective * I_stator. Supply Current = Power / V_batt.
      // Retains correct signage for regenerative braking.
      double electricalPowerW = effectiveVoltage * currentDrawAmps;
      double supplyCurrentAmps = electricalPowerW / safeBatteryVoltage;
      totalCurrentAmps += supplyCurrentAmps;

      double motorTorqueNm = motor.getTorque(currentDrawAmps);

      // Calculate Stator Thermals (I^2 * (R / C_thermal)) and ambient radiation
      // A Kraken X60 has ~200 J/K thermal mass. R = 0.03 Ohms. Constant = 0.03 / 200 = 0.00015
      double heatGenerated = (currentDrawAmps * currentDrawAmps * 0.00015) * dtSeconds;
      double heatDissipated = (statorTempCelsius[i] - 25.0) * 0.00075 * dtSeconds;
      statorTempCelsius[i] = Math.max(25.0, statorTempCelsius[i] + heatGenerated - heatDissipated);

      // Hardware-mimicking Thermal Torque Rollback (Fade output over 90C)
      if (statorTempCelsius[i] > 90.0) {
        double fadeFactor = Math.max(0.1, 1.0 - ((statorTempCelsius[i] - 90.0) * 0.05));
        motorTorqueNm *= fadeFactor;
      }

      double wheelTorqueNm = motorTorqueNm * SwerveConstants.DRIVE_GEAR_RATIO;

      // --- 2. TIRE SLIP PHYSICS (Implicit Friction Model) ---
      double R = SwerveConstants.WHEEL_RADIUS_METERS;
      double wheelSurfaceSpeedMps = simulatedWheelOmegas[i] * R;
      double longitudinalSlipMps = wheelSurfaceSpeedMps - groundSpeedMps;

      // Select friction coefficient based on slip magnitude
      double cofEffective =
          (Math.abs(longitudinalSlipMps) > SwerveConstants.SLIP_TRANSITION_VELOCITY_MPS)
              ? SwerveConstants.WHEEL_COF_KINETIC
              : SwerveConstants.WHEEL_COF_STATIC;
      double maxTractiveForceN = normalForceN * cofEffective;

      // Calculate torque needed to perfectly grip the carpet this tick (slip = 0)
      double groundOmega = groundSpeedMps / R;
      double requiredFrictionTorqueNm =
          (SwerveConstants.WHEEL_MOI_KG_M2 * (groundOmega - simulatedWheelOmegas[i]) / dtSeconds)
              - wheelTorqueNm;

      // T = -F * r => F = -T / r
      double requiredTractiveForceN = -requiredFrictionTorqueNm / R;
      double tractiveForceN;

      if (Math.abs(requiredTractiveForceN) <= maxTractiveForceN) {
        // Grip achieved (Static Friction)
        tractiveForceN = requiredTractiveForceN;
        simulatedWheelOmegas[i] = groundOmega;
      } else {
        // Grip broken (Kinetic Friction / Burnout)
        tractiveForceN = Math.copySign(maxTractiveForceN, requiredTractiveForceN);
        double loadTorqueNm = -tractiveForceN * R;
        double wheelAlphaRadPerSec2 =
            (wheelTorqueNm + loadTorqueNm) / SwerveConstants.WHEEL_MOI_KG_M2;
        simulatedWheelOmegas[i] += wheelAlphaRadPerSec2 * dtSeconds;
      }

      // Apply tractive force to chassis
      Vector2 tractiveForceWorld =
          new Vector2(moduleForward.x * tractiveForceN, moduleForward.y * tractiveForceN);

      // --- 3. LATERAL SLIP FRICTION (Implicit) ---
      double lateralSlipMps = moduleVelWorld.dot(moduleRight);

      // Calculate force needed to halt lateral sliding.
      // We use an effective mass of 8.5kg per wheel instead of M/4 (15.8kg).
      // Using M/4 applies a rotational torque that over-corrects angular slip by 168% per tick,
      // creating a severe high-frequency jitter that averages out to zero grip (floaty).
      // 8.5kg perfectly balances 53% translational correction and 90% rotational correction per
      // tick.
      double effectiveMassPerWheelKG = 8.5;
      double requiredLateralForceN = -lateralSlipMps * effectiveMassPerWheelKG / dtSeconds;

      double lateralForceN = requiredLateralForceN;
      if (Math.abs(lateralForceN) > maxTractiveForceN) {
        lateralForceN = Math.copySign(maxTractiveForceN, lateralForceN);
      }

      Vector2 lateralForceWorld =
          new Vector2(moduleRight.x * lateralForceN, moduleRight.y * lateralForceN);

      // Sum and apply at the module contact point
      Vector2 totalForceWorld = tractiveForceWorld.add(lateralForceWorld);
      chassisBody.applyForce(totalForceWorld, chassisBody.getWorldCenter().copy().add(offsetWorld));

      // Telemetry bookkeeping
      tractiveForcesN[i] = tractiveForceN;
      lateralForcesN[i] = Math.abs(lateralForceN);
      wheelRPMs[i] = Units.radiansPerSecondToRotationsPerMinute(simulatedWheelOmegas[i]);
    }

    // Export per-module telemetry to AdvantageKit
    Logger.recordOutput("PhysicsSim/TractiveForces_N", tractiveForcesN);
    Logger.recordOutput("PhysicsSim/LateralSlipForces_N", lateralForcesN);
    Logger.recordOutput("PhysicsSim/WheelRPMs", wheelRPMs);
    Logger.recordOutput("PhysicsSim/BatteryVoltage", batteryVoltage);
    Logger.recordOutput("PhysicsSim/TotalCurrentDraw_A", totalCurrentAmps);
    Logger.recordOutput("PhysicsSim/StatorTemp_C", statorTempCelsius);

    MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(totalCurrentAmps);
  }

  /**
   * Returns the simulated angular velocity of a specific wheel (rad/s).
   *
   * <p>Used by {@link com.marslib.swerve.SwerveModuleIOSim} to feed back physically accurate wheel
   * speeds into the odometry pipeline, ensuring a single source of truth for wheel dynamics.
   *
   * @param moduleIndex The module index (0=FL, 1=FR, 2=BL, 3=BR).
   * @return The wheel's angular velocity in rad/s at the output shaft (post-gearing).
   */
  public double getWheelOmegaRadPerSec(int moduleIndex) {
    return simulatedWheelOmegas[moduleIndex];
  }

  /**
   * Returns the actual chassis velocity after dyn4j has resolved all collisions for the current
   * timestep.
   *
   * @return The physics-constrained {@link ChassisSpeeds} in field-relative coordinates.
   */
  public ChassisSpeeds getConstrainedSpeeds() {
    Vector2 linVel = chassisBody.getLinearVelocity();
    double angVel = chassisBody.getAngularVelocity();
    return new ChassisSpeeds(linVel.x, linVel.y, angVel);
  }

  /**
   * Returns the current absolute pose of the chassis body in the physics world.
   *
   * @return The chassis {@link Pose2d} in standard WPILib field coordinates (meters, radians).
   */
  public Pose2d getPose() {
    return new Pose2d(
        chassisBody.getTransform().getTranslationX(),
        chassisBody.getTransform().getTranslationY(),
        new Rotation2d(chassisBody.getTransform().getRotationAngle()));
  }

  /**
   * Teleports the chassis to a new pose and zeros all velocities.
   *
   * <p>This is a hard reset—all linear, angular, and wheel velocities are zeroed. Used when the
   * odometry origin is reset (e.g., at the start of autonomous).
   *
   * @param pose The new field-relative pose to place the chassis at.
   */
  public void setPose(Pose2d pose) {
    chassisBody.getTransform().setTranslation(pose.getX(), pose.getY());
    chassisBody.getTransform().setRotation(pose.getRotation().getRadians());
    chassisBody.setLinearVelocity(0, 0);
    chassisBody.setAngularVelocity(0);

    for (int i = 0; i < NUM_MODULES; i++) {
      simulatedWheelOmegas[i] = 0.0;
    }
  }
}
