package com.marslib.power;

import com.marslib.simulation.MARSPhysicsWorld;

public class PowerIOSim implements PowerIO {

  @Override
  public void updateInputs(PowerIOInputs inputs) {
    inputs.voltage = MARSPhysicsWorld.getInstance().getSimulatedVoltage();
    inputs.totalCurrentAmps = 0.0;
    inputs.channelCurrentsAmps = new double[24];
  }
}
