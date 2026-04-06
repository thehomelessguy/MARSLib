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

  public LinearMechanismIOSim(
      String mechanismName, double massKg, double gearRatio, double spoolRadiusMeters) {
    this.gearRatio = gearRatio;
    this.spoolRadiusMeters = spoolRadiusMeters;
    this.gearbox = DCMotor.getKrakenX60Foc(1);

    // Anchor body (Static base of elevator)
    anchorBody = new Body();
    anchorBody.addFixture(Geometry.createRectangle(0.5, 0.1));
    anchorBody.setMass(MassType.INFINITE);
    anchorBody.translate(0.0, 0.0);

    // Carriage body (Dynamic)
    carriageBody = new Body();
    carriageBody.addFixture(Geometry.createRectangle(0.5, 0.1), massKg / 0.05, 0.2, 0.0);
    carriageBody.setMass(MassType.NORMAL);
    carriageBody.translate(0.0, 0.0);

    // Joint allows only vertical translation
    joint =
        new PrismaticJoint<Body>(
            anchorBody, carriageBody, new Vector2(0.0, 0.0), new Vector2(0.0, 1.0));

    MARSPhysicsWorld.getInstance().getWorld().addBody(anchorBody);
    MARSPhysicsWorld.getInstance().registerMechanismBody(mechanismName, carriageBody);
    MARSPhysicsWorld.getInstance().getWorld().addJoint(joint);

    internalController =
        new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 4.0));
  }

  @Override
  public void updateInputs(LinearMechanismIOInputs inputs) {
    // Current state mappings. Elevator only moves in Y axis perfectly.
    double currentPosMeters = carriageBody.getTransform().getTranslationY();
    double currentVelocityMetersPerSec = carriageBody.getLinearVelocity().y;

    if (closedLoop) {
      double pidVolts = internalController.calculate(currentPosMeters);
      appliedVolts = pidVolts + currentFeedforward;
      appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
    }

    // DC Motor Math linearly coupled via spool
    double motorSpeedRadsPerSec = (currentVelocityMetersPerSec / spoolRadiusMeters) * gearRatio;
    double currentDrawAmps = gearbox.getCurrent(motorSpeedRadsPerSec, appliedVolts);
    double motorTorque = gearbox.getTorque(currentDrawAmps);

    // Torque / radius = Linear Force
    double linearForceNewtons = (motorTorque * gearRatio) / spoolRadiusMeters;

    // Apply linear force directly to the body upwards
    carriageBody.applyForce(new Vector2(0.0, linearForceNewtons));

    MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(Math.abs(currentDrawAmps));

    inputs.hasHardwareConnected = true;
    inputs.positionMeters = currentPosMeters;
    inputs.velocityMetersPerSec = currentVelocityMetersPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(currentDrawAmps)};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
  }

  @Override
  public void setClosedLoopPosition(double positionMeters, double feedforwardVolts) {
    closedLoop = true;
    internalController.setGoal(positionMeters);
    currentFeedforward = feedforwardVolts;
  }
}
