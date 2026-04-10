package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.marslib.mechanisms.FlywheelIO;
import com.marslib.mechanisms.FlywheelIOInputsAutoLogged;
import com.marslib.mechanisms.FlywheelIOSim;
import com.marslib.mechanisms.FlywheelIOTalonFX;
import com.marslib.power.MARSPowerManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the robot's intake roller mechanism.
 *
 * <p>The intake uses a single flywheel motor to pull game pieces into the robot. It accepts a
 * {@link FlywheelIO} implementation via dependency injection, allowing seamless switching between
 * real hardware ({@link FlywheelIOTalonFX}) and physics simulation ({@link FlywheelIOSim}).
 *
 * <p>Students: Bind {@link #runIntakeCommand()} and {@link #runReverseCommand()} to controller
 * buttons in {@code RobotContainer} for basic intake/outtake control.
 */
public class MARSIntake extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // Bounds for active load shedding
  private static final double NOMINAL_VOLTAGE = 12.0;
  private static final double CRITICAL_VOLTAGE = 9.0;
  private static final double MAX_CURRENT_AMPS = 40.0;
  private static final double MIN_CURRENT_AMPS = 20.0;

  private final MARSPowerManager powerManager;
  private final SysIdRoutine sysIdRoutine;

  /**
   * Constructs the intake subsystem.
   *
   * @param io The hardware abstraction layer for the intake roller motor.
   * @param powerManager The active power manager for load-shedding voltage queries.
   */
  public MARSIntake(FlywheelIO io, MARSPowerManager powerManager) {
    this.io = io;
    this.powerManager = powerManager;

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
    Logger.processInputs("Intake", inputs);

    // Active Load Shedding via MARSPowerManager
    double currentLimit =
        powerManager.calculateLoadSheddedLimit(
            MAX_CURRENT_AMPS, MIN_CURRENT_AMPS, NOMINAL_VOLTAGE, CRITICAL_VOLTAGE);

    io.setCurrentLimit(currentLimit);
    Logger.recordOutput("Intake/ActiveCurrentLimit", currentLimit);
  }

  /**
   * Returns a command that runs the intake forward at full voltage, stopping when cancelled.
   *
   * @return A {@link Command} that spins the roller inward at 12V.
   */
  public Command runIntakeCommand() {
    return this.startEnd(() -> io.setVoltage(12.0), () -> io.setVoltage(0.0));
  }

  /**
   * Returns a command that runs the intake in reverse at full voltage, stopping when cancelled.
   *
   * @return A {@link Command} that spins the roller outward at -12V for ejecting jammed pieces.
   */
  public Command runReverseCommand() {
    return this.startEnd(() -> io.setVoltage(-12.0), () -> io.setVoltage(0.0));
  }

  /**
   * Directly sets the intake motor voltage. Intended for use by the {@link MARSSuperstructure}.
   *
   * @param volts The voltage to apply (positive = inward, negative = outward).
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the intake motor immediately. */
  public void stop() {
    io.setVoltage(0.0);
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
}
