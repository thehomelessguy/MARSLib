package com.marslib.mechanisms;

import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.PrismaticJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

/**
 * Simulation implementation of the LinearMechanismIO interface. Uses Dyn4j to run a rigid-body
 * physics simulation of a 1D elevator mechanism.
 */
public class LinearMechanismIOSim implements LinearMechanismIO {

  private final Body carriageBody;
  private final Body anchorBody;
  private final PrismaticJoint<Body> joint;

  private final DCMotor gearbox;
  private final double gearRatio;
  private final double spoolRadiusMeters;

  private final ProfiledPIDController internalController;

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double currentFeedforward = 0.0;

  /**
   * Constructs a physical simulation instance for a 1D linear elevator mechanism.
   *
   * <p>This models the mechanism as a Dyn4j prismatic joint and accurately calculates forces based
   * on an attached Kraken X60 motor and the physical spool parameters.
   *
   * @param mechanismName The unique identifier used for AdvantageKit logging (e.g. "Elevator").
   * @param gearRatio The total gear reduction from motor to spool (e.g., 25.0 for a 25:1
   *     reduction).
   * @param spoolDiameterMeters The diameter of the driven spool in meters.
   * @param massKg The physical mass of the moving carriage and its load in kilograms.
   */
  public LinearMechanismIOSim(
      String mechanismName, double gearRatio, double spoolDiameterMeters, double massKg) {
    this.gearRatio = gearRatio;
    this.spoolRadiusMeters = spoolDiameterMeters / 2.0;
    this.gearbox = DCMotor.getKrakenX60Foc(1);

    // Anchor body (Static base of elevator)
    anchorBody = new Body();
    anchorBody.addFixture(Geometry.createRectangle(0.5, 0.1));
    anchorBody.setMass(MassType.INFINITE);
    anchorBody.translate(0.0, 5.0);

    // Carriage body (Dynamic)
    carriageBody = new Body();
    // Typical FRC elevator carriage geometry approximation
    double carriageWidth = 0.5;
    double carriageHeight = 0.1;
    double area = carriageWidth * carriageHeight;
    carriageBody.addFixture(
        Geometry.createRectangle(carriageWidth, carriageHeight), massKg / area, 0.2, 0.0);
    carriageBody.setMass(MassType.NORMAL);
    carriageBody.translate(0.0, 5.0);

    // Joint allows only vertical translation
    joint =
        new PrismaticJoint<Body>(
            anchorBody, carriageBody, new Vector2(0.0, 5.0), new Vector2(0.0, 1.0));
    joint.setCollisionAllowed(false);

    MARSPhysicsWorld.getInstance().getWorld().addBody(anchorBody);
    MARSPhysicsWorld.getInstance().registerMechanismBody(mechanismName, carriageBody);
    MARSPhysicsWorld.getInstance().getWorld().addJoint(joint);

    // Internal profiled PID mimics the TalonFX Motion Magic controller in sim.
    // kP=500.0 provides stiff tracking without ff; constraints model FRC elevator profile:
    //   maxVelocity = 2.0 m/s, maxAcceleration = 4.0 m/s²
    internalController =
        new ProfiledPIDController(500.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 4.0));
  }

  @Override
  public void updateInputs(LinearMechanismIOInputs inputs) {
    // Current state mappings. Elevator only moves in Y axis perfectly. Offset anchor height 5.0m
    double currentPosMeters = carriageBody.getTransform().getTranslationY() - 5.0;
    double currentVelocityMetersPerSec = carriageBody.getLinearVelocity().y;

    if (closedLoop) {
      double pidVolts = internalController.calculate(currentPosMeters);
      appliedVolts = pidVolts + currentFeedforward;
      double maxVoltage = Math.max(MARSPhysicsWorld.getInstance().getSimulatedVoltage(), 0.01);
      appliedVolts = Math.max(-maxVoltage, Math.min(maxVoltage, appliedVolts));
    }

    // DC Motor Math linearly coupled via spool
    double motorSpeedRadsPerSec = (currentVelocityMetersPerSec / spoolRadiusMeters) * gearRatio;
    double currentDrawAmps = gearbox.getCurrent(motorSpeedRadsPerSec, appliedVolts);
    // Enforce stator current limit like real TalonFX firmware
    currentDrawAmps = Math.copySign(Math.min(Math.abs(currentDrawAmps), 40.0), currentDrawAmps);
    double motorTorque = gearbox.getTorque(currentDrawAmps);

    // Torque / radius = Linear Force
    double linearForceNewtons = (motorTorque * gearRatio) / spoolRadiusMeters;

    // Apply linear force directly to the body upwards
    carriageBody.applyForce(new Vector2(0.0, linearForceNewtons));

    // Compute effective motor terminal voltage after current limiting
    // V_effective = I·R + ω/Kv (what the motor controller actually applies)
    double batteryVoltage = Math.max(MARSPhysicsWorld.getInstance().getSimulatedVoltage(), 0.01);

    double effectiveVoltage =
        currentDrawAmps * gearbox.rOhms + motorSpeedRadsPerSec / gearbox.KvRadPerSecPerVolt;
    if (Math.abs(effectiveVoltage) > batteryVoltage) {
      effectiveVoltage = Math.copySign(batteryVoltage, effectiveVoltage);
      currentDrawAmps =
          (effectiveVoltage - motorSpeedRadsPerSec / gearbox.KvRadPerSecPerVolt) / gearbox.rOhms;
    }

    double electricalPowerW = effectiveVoltage * currentDrawAmps;
    double supplyCurrentAmps = electricalPowerW / batteryVoltage;

    MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(supplyCurrentAmps);

    inputs.hasHardwareConnected = true;
    inputs.positionMeters = currentPosMeters;
    inputs.velocityMetersPerSec = currentVelocityMetersPerSec;
    // Feed back the profiled setpoint velocity so the subsystem-level ElevatorFeedforward
    // kV term is exercised in simulation (mirrors MotionMagic reference slope on real HW).
    inputs.targetVelocityMetersPerSec =
        closedLoop ? internalController.getSetpoint().velocity : 0.0;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(currentDrawAmps)};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    double maxVoltage = MARSPhysicsWorld.getInstance().getSimulatedVoltage();
    appliedVolts = Math.max(-maxVoltage, Math.min(maxVoltage, volts));
  }

  @Override
  public void setClosedLoopPosition(double positionMeters, double feedforwardVolts) {
    closedLoop = true;
    internalController.setGoal(positionMeters);
    currentFeedforward = feedforwardVolts;
  }
}
