package com.marslib.power;

import com.marslib.simulation.MARSPhysicsWorld;

/** Simulated implementation of the power and battery IO interface. */
public class PowerIOSim implements PowerIO {

  @Override
  public void updateInputs(PowerIOInputs inputs) {
    inputs.voltage = MARSPhysicsWorld.getInstance().getSimulatedVoltage();
    inputs.totalCurrentAmps = 0.0;
    inputs.channelCurrentsAmps = new double[24];

    // Simulate generic CAN utilization between 65% and 80%
    inputs.canBusUtilization = 0.65 + (Math.random() * 0.15);

    if (frc.robot.Constants.SimulationConstants.ENABLE_CAN_STARVATION) {
      if (Math.random() < frc.robot.Constants.SimulationConstants.CAN_STARVATION_PROBABILITY) {
        inputs.canBusUtilization = 1.0;
        try {
          Thread.sleep(frc.robot.Constants.SimulationConstants.CAN_STARVATION_DELAY_MS);
        } catch (InterruptedException e) {
        }
      }
    }

    // Explicitly flag if the physics world dynamically sank our voltage below real-life roboRio
    // limits
    inputs.isBrownedOut = inputs.voltage < frc.robot.Constants.PowerConstants.CRITICAL_VOLTAGE;
  }
}
