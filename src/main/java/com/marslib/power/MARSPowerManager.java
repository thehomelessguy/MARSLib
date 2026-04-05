package com.marslib.power;

import com.marslib.faults.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MARSPowerManager extends SubsystemBase {
  private final PowerIO io;
  private final PowerIOInputsAutoLogged inputs = new PowerIOInputsAutoLogged();

  private final Alert warningAlert =
      new Alert("Power", "Load Shedding: Voltage below 8.0V", Alert.AlertType.WARNING);
  private final Alert criticalAlert =
      new Alert("Power", "Load Shedding: Voltage below 7.0V", Alert.AlertType.CRITICAL);

  public MARSPowerManager(PowerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Power", inputs);

    if (inputs.voltage > 0) {
      if (inputs.voltage < 7.0) {
        warningAlert.set(true);
        criticalAlert.set(true);
      } else if (inputs.voltage < 8.0) {
        warningAlert.set(true);
        criticalAlert.set(false);
      } else {
        warningAlert.set(false);
        criticalAlert.set(false);
      }
    } else {
      warningAlert.set(false);
      criticalAlert.set(false);
    }
  }
}
