package com.marslib.power;

import edu.wpi.first.wpilibj.PowerDistribution;

/** Hardware implementation for reading physical Power Distribution Hub (PDH) data. */
public class PowerIOReal implements PowerIO {
  private final PowerDistribution pdh;

  public PowerIOReal() {
    pdh = new PowerDistribution();
  }

  @Override
  public void updateInputs(PowerIOInputs inputs) {
    inputs.totalCurrentAmps = pdh.getTotalCurrent();
    inputs.voltage = pdh.getVoltage();

    int numChannels = pdh.getNumChannels();
    if (inputs.channelCurrentsAmps.length != numChannels) {
      inputs.channelCurrentsAmps = new double[numChannels];
    }
    for (int i = 0; i < numChannels; i++) {
      inputs.channelCurrentsAmps[i] = pdh.getCurrent(i);
    }
  }
}
