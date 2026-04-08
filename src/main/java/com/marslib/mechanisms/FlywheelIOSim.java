package com.marslib.mechanisms;

import com.marslib.simulation.MARSPhysicsWorld;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of the FlywheelIO interface. Uses WPILib's FlywheelSim to model 1D
 * rotational dynamics and integrates with the centralized MARSPhysicsWorld electrical model for
 * realistic battery sag accurately mapped to current draw.
 */
public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim sim;
  private final DCMotor gearbox;
  private final PIDController controller;

  private double appliedVolts = 0.0;
  private double targetVelocityRadPerSec = 0.0;

  /**
   * Constructs a physical simulation instance for a 1D flywheel mechanism.
   *
   * @param gearbox The WPILib DCMotor model representing the physical motor (e.g. Kraken X60).
   * @param gearing The total gear reduction from motor to flywheel.
   * @param momentOfInertiaKgMetersSquared The moment of inertia of the rotating flywheel in kg*m^2.
   */
  public FlywheelIOSim(DCMotor gearbox, double gearing, double momentOfInertiaKgMetersSquared) {
    this.gearbox = gearbox;
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

    // Compute effective motor terminal voltage after current limiting
    double statorCurrent = sim.getCurrentDrawAmps();
    double batteryVoltage = Math.max(MARSPhysicsWorld.getInstance().getSimulatedVoltage(), 0.01);
    double currentLimitAmps = 40.0;
    boolean currentLimited = Math.abs(statorCurrent) > currentLimitAmps;
    double clampedCurrent =
        Math.copySign(Math.min(Math.abs(statorCurrent), currentLimitAmps), statorCurrent);

    double effectiveVoltage =
        clampedCurrent * gearbox.rOhms
            + sim.getAngularVelocityRadPerSec() / gearbox.KvRadPerSecPerVolt;
    if (Math.abs(effectiveVoltage) > batteryVoltage) {
      effectiveVoltage = Math.copySign(batteryVoltage, effectiveVoltage);
      clampedCurrent =
          (effectiveVoltage - sim.getAngularVelocityRadPerSec() / gearbox.KvRadPerSecPerVolt)
              / gearbox.rOhms;
    }

    double electricalPowerW = effectiveVoltage * clampedCurrent;
    double supplyCurrentAmps = electricalPowerW / batteryVoltage;

    MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(supplyCurrentAmps);

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
    System.out.println(
        "[SIM_INPUT] applying volts="
            + appliedVolts
            + "; PID="
            + pidVolts
            + " target="
            + velocityRadPerSec);
    sim.setInputVoltage(appliedVolts);
  }
}
