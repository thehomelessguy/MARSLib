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
  private static final double NOMINAL_VOLTAGE = 12.0;
  private static final double CRITICAL_VOLTAGE = 9.0;

  private static final double MAX_CURRENT_AMPS = 40.0;
  private static final double MIN_CURRENT_AMPS = 20.0;

  private final MARSPowerManager powerManager;
  private final SysIdRoutine sysIdRoutine;

  public MARSArm(RotaryMechanismIO io, MARSPowerManager powerManager) {
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
    Logger.processInputs("Arm", inputs);

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
    Logger.recordOutput("Arm/ActiveCurrentLimit", currentLimit);
  }

  private double targetPositionRads = 0.0;

  /**
   * Commands the arm to a target angular position using Motion Magic with dynamic feedforward.
   *
   * <p>The feedforward voltage is computed using the current arm angle (for gravity compensation)
   * and the instantaneous Motion Magic profile velocity (for kV contribution).
   *
   * @param positionRads Target arm angle in radians.
   */
  public void setTargetPosition(double positionRads) {
    this.targetPositionRads = positionRads;
    // Dynamic FF using actual arm physical angle and instantaneous profile target velocity from
    // CTRE Motion Magic
    double currentAngleRads = inputs.positionRad;
    double ffVolts = feedforward.calculate(currentAngleRads, inputs.targetVelocityRadPerSec);
    io.setClosedLoopPosition(positionRads, ffVolts);
    Logger.recordOutput("Arm/TargetPositionRads", positionRads);
  }

  public boolean isAtTolerance() {
    return Math.abs(inputs.positionRad - targetPositionRads)
        < edu.wpi.first.math.util.Units.degreesToRadians(3.0);
  }

  /**
   * Returns the current measured arm angle.
   *
   * @return Current arm position in radians.
   */
  public double getPositionRads() {
    return inputs.positionRad;
  }

  /**
   * Generates a SysId Quasistatic characterization command.
   *
   * @param direction The direction of the quasistatic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Generates a SysId Dynamic characterization command.
   *
   * @param direction The direction of the dynamic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Homes the arm by driving at a constant voltage and checking the stator current. It resets the
   * encoder to 0.0 radians upon reaching the hard stop.
   *
   * @param homingVoltage Voltage to drive the motor (negative for downwards).
   * @param currentThresholdAmps Stator current threshold (i.e. 30.0).
   * @return The home command.
   */
  public Command homeWithCurrent(double homingVoltage, double currentThresholdAmps) {
    return run(() -> io.setVoltage(homingVoltage))
        .until(
            () -> {
              // Check if any motor exceeds the current threshold
              for (double current : inputs.currentAmps) {
                if (current > currentThresholdAmps) return true;
              }
              return false;
            })
        .andThen(
            () -> {
              io.setVoltage(0.0);
              io.setEncoderPosition(0.0);
              Logger.recordOutput("MARSArm/Status", "Homing Complete!");
            });
  }

  /**
   * Homes the arm using a standard default threshold (-2.0V, 15.0A).
   *
   * @return The home command.
   */
  public Command home() {
    return homeWithCurrent(-2.0, 15.0);
  }
}
