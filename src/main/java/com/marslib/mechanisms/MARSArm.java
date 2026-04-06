package com.marslib.mechanisms;

import com.marslib.power.MARSPowerManager;
import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * High-level subsystem representing a rotating single-jointed arm.
 *
 * <p>Handles physics simulation, angular position tracking, and dynamic load shedding to prevent
 * battery brownouts using MARSPowerManager data.
 */
public class MARSArm extends SubsystemBase {

  private final RotaryMechanismIO io;
  private final RotaryMechanismIOInputsAutoLogged inputs = new RotaryMechanismIOInputsAutoLogged();

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", 0.0);

  private ArmFeedforward feedforward;

  // Bounds for active load shedding
  private static final double NOMINAL_VOLTAGE = Constants.ArmConstants.NOMINAL_VOLTAGE;
  private static final double CRITICAL_VOLTAGE = Constants.ArmConstants.CRITICAL_VOLTAGE;

  private static final double MAX_CURRENT_AMPS = Constants.ArmConstants.MAX_CURRENT_AMPS;
  private static final double MIN_CURRENT_AMPS = Constants.ArmConstants.MIN_CURRENT_AMPS;

  private final MARSPowerManager powerManager;

  public MARSArm(RotaryMechanismIO io, MARSPowerManager powerManager) {
    this.io = io;
    this.powerManager = powerManager;
    feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update Feedforward if TUNING mode constants are changed
    boolean sChanged = kS.hasChanged(kS.hashCode());
    boolean gChanged = kG.hasChanged(kG.hashCode());
    boolean vChanged = kV.hasChanged(kV.hashCode());
    boolean aChanged = kA.hasChanged(kA.hashCode());

    if (sChanged || gChanged || vChanged || aChanged) {
      feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    // Active Load Shedding via MARSPowerManager
    double sysVoltage = powerManager.getVoltage();
    double currentLimit = MAX_CURRENT_AMPS;

    if (sysVoltage < NOMINAL_VOLTAGE) {
      double slope = (MAX_CURRENT_AMPS - MIN_CURRENT_AMPS) / (NOMINAL_VOLTAGE - CRITICAL_VOLTAGE);
      currentLimit = MIN_CURRENT_AMPS + slope * (sysVoltage - CRITICAL_VOLTAGE);
      currentLimit = Math.max(MIN_CURRENT_AMPS, Math.min(MAX_CURRENT_AMPS, currentLimit));
    }

    io.setCurrentLimit(currentLimit);
    Logger.recordOutput("Arm/ActiveCurrentLimit", currentLimit);
  }

  public void setTargetPosition(double positionRads) {
    // Dynamic FF using actual arm physical angle and instantaneous profile target velocity from
    // CTRE Motion Magic
    double currentAngleRads = inputs.positionRad;
    double ffVolts = feedforward.calculate(currentAngleRads, inputs.targetVelocityRadPerSec);
    io.setClosedLoopPosition(positionRads, ffVolts);
    Logger.recordOutput("Arm/TargetPositionRads", positionRads);
  }

  public double getPositionRads() {
    return inputs.positionRad;
  }
}
