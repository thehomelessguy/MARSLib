package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.marslib.mechanisms.*;
import com.marslib.power.MARSPowerManager;
import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem representing the robot's fast climber mechanism.
 *
 * <p>Handles physics simulation, linear position tracking, and dynamic load shedding to prevent
 * battery brownouts using MARSPowerManager data.
 */
public class MARSClimber extends SubsystemBase {

  private final LinearMechanismIO io;
  private final LinearMechanismIOInputsAutoLogged inputs = new LinearMechanismIOInputsAutoLogged();

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Climber/kS", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Climber/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Climber/kA", 0.0);

  private ElevatorFeedforward feedforward;

  // Bounds for active load shedding
  private static final double NOMINAL_VOLTAGE = 12.0;
  private static final double CRITICAL_VOLTAGE = 9.0;

  private static final double MAX_CURRENT_AMPS = 40.0;
  private static final double MIN_CURRENT_AMPS = 20.0;

  private final MARSPowerManager powerManager;
  private final SysIdRoutine sysIdRoutine;

  public MARSClimber(LinearMechanismIO io, MARSPowerManager powerManager) {
    this.io = io;
    this.powerManager = powerManager;
    feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

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
    Logger.processInputs("Climber", inputs);

    // Update Feedforward if TUNING mode constants are changed
    int id = this.hashCode();
    boolean sChanged = kS.hasChanged(id);
    boolean gChanged = kG.hasChanged(id);
    boolean vChanged = kV.hasChanged(id);
    boolean aChanged = kA.hasChanged(id);

    if (sChanged || gChanged || vChanged || aChanged) {
      feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    // Active Load Shedding via MARSPowerManager
    double currentLimit =
        powerManager.calculateLoadSheddedLimit(
            MAX_CURRENT_AMPS, MIN_CURRENT_AMPS, NOMINAL_VOLTAGE, CRITICAL_VOLTAGE);

    io.setCurrentLimit(currentLimit);
    Logger.recordOutput("Climber/ActiveCurrentLimit", currentLimit);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Commands the climber to a target height using Motion Magic with dynamic feedforward.
   *
   * @param positionMeters Target elevator height in meters.
   */
  public void setTargetPosition(double positionMeters) {
    // Dynamic FF using instantaneous profile target velocity from CTRE Motion Magic
    double ffVolts = feedforward.calculate(inputs.targetVelocityMetersPerSec);
    io.setClosedLoopPosition(positionMeters, ffVolts);
    Logger.recordOutput("Climber/TargetPositionMeters", positionMeters);
  }

  /**
   * Returns the current measured climber height.
   *
   * @return Current position in meters.
   */
  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
