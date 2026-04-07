package com.marslib.mechanisms;

import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

/**
 * Simulation implementation of the RotaryMechanismIO interface. Uses Dyn4j to run a rigid-body
 * physics simulation of a 1D rotary arm mechanism.
 */
public class RotaryMechanismIOSim implements RotaryMechanismIO {

  private final Body armBody;
  private final Body anchorBody;
  private final RevoluteJoint<Body> joint;

  private final DCMotor gearbox;
  private final double gearRatio;

  private final ProfiledPIDController internalController;

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double currentFeedforward = 0.0;

  /**
   * Constructs a physical simulation instance for a 1D rotary arm mechanism.
   *
   * <p>This models the mechanism as a Dyn4j revolute joint and accurately calculates rotational
   * inertia and forces based on an attached Kraken X60 motor.
   *
   * @param mechanismName The unique identifier used for AdvantageKit logging (e.g. "Arm").
   * @param gearRatio The total gear reduction from motor to arm axis.
   * @param jKgMetersSquared The total moment of inertia of the rotating arm in kg*m^2.
   * @param lengthMeters The physical extended length of the arm for visualization and density
   *     mapping.
   */
  public RotaryMechanismIOSim(
      String mechanismName, double gearRatio, double jKgMetersSquared, double lengthMeters) {
    this.gearRatio = gearRatio;
    this.gearbox = DCMotor.getKrakenX60Foc(1);

    // Anchor body (Static)
    anchorBody = new Body();
    anchorBody.addFixture(Geometry.createRectangle(0.1, 0.1));
    anchorBody.setMass(MassType.INFINITE);
    anchorBody.translate(0.0, 0.0);

    // Arm body (Dynamic)
    armBody = new Body();
    double thickness = 0.05;
    // Calculate mass from moment of inertia assuming rigid rod rotating about its center
    // I = (1/12) * m * L^2  =>  m = 12 * I / L^2
    double mass = 12.0 * jKgMetersSquared / (lengthMeters * lengthMeters);
    double area = lengthMeters * thickness;
    double density = mass / area;
    armBody.addFixture(
        Geometry.createRectangle(lengthMeters, thickness), density, 0.2, 0.0); // Simple rod
    armBody.setMass(MassType.NORMAL);
    // Center mass roughly half way
    armBody.translate(lengthMeters / 2.0, 0.0);

    // Joint binds at origin
    joint = new RevoluteJoint<Body>(anchorBody, armBody, new Vector2(0.0, 0.0));

    // Collision filtering to prevent the arm colliding with abstract simulation bounds (e.g. floor)
    anchorBody.getFixture(0).setFilter(new CategoryFilter(4, 4));
    armBody.getFixture(0).setFilter(new CategoryFilter(4, 4));

    // Register into the singleton Physics world
    MARSPhysicsWorld.getInstance().getWorld().addBody(anchorBody);
    MARSPhysicsWorld.getInstance().registerMechanismBody(mechanismName, armBody);
    MARSPhysicsWorld.getInstance().getWorld().addJoint(joint);

    // Internal profiled PID mimics the TalonFX Motion Magic controller in sim.
    // kP=5.0 provides stiff tracking; constraints model a typical FRC arm profile:
    //   maxVelocity = 10.0 rad/s, maxAcceleration = 20.0 rad/s²
    internalController =
        new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 20.0));
  }

  @Override
  public void updateInputs(RotaryMechanismIOInputs inputs) {
    double currentAngleRad = armBody.getTransform().getRotationAngle();
    double currentVelocityRadPerSec = armBody.getAngularVelocity();

    if (closedLoop) {
      double pidVolts = internalController.calculate(currentAngleRad);
      appliedVolts = pidVolts + currentFeedforward;
      double maxVoltage = Math.max(MARSPhysicsWorld.getInstance().getSimulatedVoltage(), 0.01);
      appliedVolts = Math.max(-maxVoltage, Math.min(maxVoltage, appliedVolts));
    }

    // DC Motor Math
    double currentDrawAmps = gearbox.getCurrent(currentVelocityRadPerSec * gearRatio, appliedVolts);
    // Enforce stator current limit like real TalonFX firmware
    currentDrawAmps = Math.copySign(Math.min(Math.abs(currentDrawAmps), 40.0), currentDrawAmps);
    double motorTorque = gearbox.getTorque(currentDrawAmps);
    double mechanismTorque = motorTorque * gearRatio;

    // Apply strictly to Dyn4j body
    armBody.applyTorque(mechanismTorque);

    System.out.println(
        "ArmSim | Pos: "
            + currentAngleRad
            + " | Vel: "
            + currentVelocityRadPerSec
            + " | Motor Torque: "
            + motorTorque
            + " | Applied Volts: "
            + appliedVolts
            + " | Target Pos: "
            + internalController.getGoal().position);

    // Compute effective motor terminal voltage after current limiting
    double motorSpeedRadPerSec = currentVelocityRadPerSec * gearRatio;
    double batteryVoltage = Math.max(MARSPhysicsWorld.getInstance().getSimulatedVoltage(), 0.01);

    double effectiveVoltage =
        currentDrawAmps * gearbox.rOhms + motorSpeedRadPerSec / gearbox.KvRadPerSecPerVolt;
    if (Math.abs(effectiveVoltage) > batteryVoltage) {
      effectiveVoltage = Math.copySign(batteryVoltage, effectiveVoltage);
      currentDrawAmps =
          (effectiveVoltage - motorSpeedRadPerSec / gearbox.KvRadPerSecPerVolt) / gearbox.rOhms;
    }

    double electricalPowerW = effectiveVoltage * currentDrawAmps;
    double supplyCurrentAmps = electricalPowerW / batteryVoltage;

    MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(supplyCurrentAmps);

    inputs.hasHardwareConnected = true;
    inputs.positionRad = currentAngleRad;
    inputs.velocityRadPerSec = currentVelocityRadPerSec;
    // Feed back the profiled setpoint velocity so the subsystem-level ArmFeedforward
    // kV term is exercised in simulation (mirrors MotionMagic reference slope on real HW).
    inputs.targetVelocityRadPerSec = closedLoop ? internalController.getSetpoint().velocity : 0.0;
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
  public void setClosedLoopPosition(double positionRad, double feedforwardVolts) {
    closedLoop = true;
    internalController.setGoal(positionRad);
    currentFeedforward = feedforwardVolts;
  }
}
