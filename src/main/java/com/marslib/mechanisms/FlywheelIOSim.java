package com.marslib.mechanisms;

import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim sim;
  private final PIDController controller;

  private double appliedVolts = 0.0;
  private double targetVelocityRadPerSec = 0.0;

  public FlywheelIOSim(DCMotor gearbox, double gearing, double momentOfInertiaKgMetersSquared) {
    this.sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(gearbox, momentOfInertiaKgMetersSquared, gearing),
            gearbox,
            gearing);
    this.controller = new PIDController(0.1, 0, 0); // basic simulated PID
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.update(0.02); // 50hz

    // Log current draw natively to physics world
    MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(sim.getCurrentDrawAmps());

    inputs.hasHardwareConnected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.targetVelocityRadPerSec = targetVelocityRadPerSec;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setClosedLoopVelocity(double velocityRadPerSec, double feedforwardVolts) {
    targetVelocityRadPerSec = velocityRadPerSec;
    double pidVolts =
        controller.calculate(sim.getAngularVelocityRadPerSec(), targetVelocityRadPerSec);
    appliedVolts = pidVolts + feedforwardVolts;
    sim.setInputVoltage(appliedVolts);
  }
}
