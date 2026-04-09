package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.marslib.mechanisms.*;
import com.marslib.power.MARSPowerManager;
import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/**
 * High-level subsystem representing the intake deployment pivot.
 *
 * <p>Handles physics simulation, angular position tracking, and dynamic load shedding to prevent
 * battery brownouts using MARSPowerManager data.
 */
public class MARSIntakePivot extends SubsystemBase {

  private final RotaryMechanismIO io;
  private final RotaryMechanismIOInputsAutoLogged inputs = new RotaryMechanismIOInputsAutoLogged();

  private final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/kS", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("IntakePivot/kG", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("IntakePivot/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("IntakePivot/kA", 0.0);

  private ArmFeedforward feedforward;

  // Bounds for active load shedding
  private static final double NOMINAL_VOLTAGE = 12.0;
  private static final double CRITICAL_VOLTAGE = 9.0;

  private static final double MAX_CURRENT_AMPS = 40.0;
  private static final double MIN_CURRENT_AMPS = 20.0;

  private final MARSPowerManager powerManager;
  private final SysIdRoutine sysIdRoutine;

  public MARSIntakePivot(RotaryMechanismIO io, MARSPowerManager powerManager) {
    this.io = io;
    this.powerManager = powerManager;
    feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (edu.wpi.first.units.measure.Voltage volts) -> {
                  io.setVoltage(volts.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    // Update Feedforward if TUNING mode constants are changed
    int id = this.hashCode();
    boolean sChanged = kS.hasChanged(id);
    boolean gChanged = kG.hasChanged(id);
    boolean vChanged = kV.hasChanged(id);
    boolean aChanged = kA.hasChanged(id);

    if (sChanged || gChanged || vChanged || aChanged) {
      feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    // Active Load Shedding via MARSPowerManager
    double currentLimit =
        powerManager.calculateLoadSheddedLimit(
            MAX_CURRENT_AMPS, MIN_CURRENT_AMPS, NOMINAL_VOLTAGE, CRITICAL_VOLTAGE);

    io.setCurrentLimit(currentLimit);
    Logger.recordOutput("IntakePivot/ActiveCurrentLimit", currentLimit);
  }

  private double targetPositionRads = 0.0;

  /**
   * Commands the intake pivot to a target angular position using Motion Magic with dynamic
   * feedforward.
   *
   * @param positionRads Target angle in radians.
   */
  public void setTargetPosition(double positionRads) {
    this.targetPositionRads = positionRads;
    double currentAngleRads = inputs.positionRad;
    double ffVolts = feedforward.calculate(currentAngleRads, inputs.targetVelocityRadPerSec);
    io.setClosedLoopPosition(positionRads, ffVolts);
    Logger.recordOutput("IntakePivot/TargetPositionRads", positionRads);
  }

  public boolean isAtTolerance() {
    return Math.abs(inputs.positionRad - targetPositionRads)
        < edu.wpi.first.math.util.Units.degreesToRadians(3.0);
  }

  public double getPositionRads() {
    return inputs.positionRad;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command homeWithCurrent(double homingVoltage, double currentThresholdAmps) {
    return run(() -> io.setVoltage(homingVoltage))
        .until(
            () -> {
              for (double current : inputs.currentAmps) {
                if (current > currentThresholdAmps) return true;
              }
              return false;
            })
        .andThen(
            () -> {
              io.setVoltage(0.0);
              io.setEncoderPosition(0.0);
              Logger.recordOutput("MARSIntakePivot/Status", "Homing Complete!");
            });
  }

  public Command home() {
    return homeWithCurrent(-2.0, 15.0);
  }
}
