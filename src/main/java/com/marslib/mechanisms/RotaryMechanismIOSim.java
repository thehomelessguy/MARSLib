package com.marslib.mechanisms;

import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

public class RotaryMechanismIOSim implements RotaryMechanismIO {

  private final Body armBody;
  private final Body anchorBody;
  private final RevoluteJoint joint;

  private final DCMotor gearbox;
  private final double gearRatio;

  private final ProfiledPIDController internalController;

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double currentFeedforward = 0.0;

  public RotaryMechanismIOSim(
      String mechanismName, double massKg, double lengthMeters, double gearRatio) {
    this.gearRatio = gearRatio;
    this.gearbox = DCMotor.getKrakenX60Foc(1);

    // Anchor body (Static)
    anchorBody = new Body();
    anchorBody.addFixture(Geometry.createRectangle(0.1, 0.1));
    anchorBody.setMass(MassType.INFINITE);
    anchorBody.translate(0.0, 0.0);

    // Arm body (Dynamic)
    armBody = new Body();
    armBody.addFixture(Geometry.createRectangle(lengthMeters, 0.05), 5.0, 0.2, 0.0); // Simple rod
    armBody.setMass(MassType.NORMAL);
    // Center mass roughly half way
    armBody.translate(lengthMeters / 2.0, 0.0);

    // Joint binds at origin
    joint = new RevoluteJoint(anchorBody, armBody, new Vector2(0.0, 0.0));

    // Register into the singleton Physics world
    MARSPhysicsWorld.getInstance().getWorld().addBody(anchorBody);
    MARSPhysicsWorld.getInstance().registerMechanismBody(mechanismName, armBody);
    MARSPhysicsWorld.getInstance().getWorld().addJoint(joint);

    // Mimic Motion Magic constraints
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
      appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
    }

    // DC Motor Math
    double currentDrawAmps = gearbox.getCurrent(currentVelocityRadPerSec * gearRatio, appliedVolts);
    double motorTorque = gearbox.getTorque(currentDrawAmps);
    double mechanismTorque = motorTorque * gearRatio;

    // Apply strictly to Dyn4j body
    armBody.applyTorque(mechanismTorque);
    MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(Math.abs(currentDrawAmps));

    inputs.hasHardwareConnected = true;
    inputs.positionRad = currentAngleRad;
    inputs.velocityRadPerSec = currentVelocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(currentDrawAmps)};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
  }

  @Override
  public void setClosedLoopPosition(double positionRad, double feedforwardVolts) {
    closedLoop = true;
    internalController.setGoal(positionRad);
    currentFeedforward = feedforwardVolts;
  }
}
