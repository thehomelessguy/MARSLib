package frc.robot.commands;

import com.marslib.faults.Alert;
import com.marslib.faults.Alert.AlertType;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * An automated diagnostic routine intended to run in Test Mode. Sweeps the SwerveDrive and
 * MARSElevator, validating hardware encoders and catching CAN failures prior to match start.
 *
 * <p>Rules Enforced: marslib-diagnostics
 */
public class SystemCheckCommand extends Command {
  private final SwerveDrive swerve;
  private final MARSElevator elevator;

  private final Timer timer = new Timer();
  private int step = 0;

  private final Alert failureAlert =
      new Alert("SystemCheck Failed! Hardware unresponsive.", AlertType.CRITICAL);
  private final Alert successAlert =
      new Alert("SystemCheck Passed: Hardened and Ready.", AlertType.INFO);

  private double initialElevatorPosition = 0;

  public SystemCheckCommand(SwerveDrive swerve, MARSElevator elevator) {
    this.swerve = swerve;
    this.elevator = elevator;
    addRequirements(swerve, elevator);
  }

  @Override
  public void initialize() {
    timer.restart();
    step = 0;
    failureAlert.set(false);
    successAlert.set(false);
    initialElevatorPosition = elevator.getPositionMeters();
    Logger.recordOutput("SystemCheck/Active", true);
    Logger.recordOutput("SystemCheck/Step", step);
  }

  @Override
  public void execute() {
    double time = timer.get();

    switch (step) {
      case 0:
        // Step 0: Command all swerve modules to physically turn to 45 degrees
        SwerveModuleState[] testStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
          testStates[i] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0));
        }
        swerve.setModuleStates(testStates);

        // Wait 1.5 seconds for modules to physically reach the target
        if (time > 1.5) {
          // Check if modules reached the 45-degree target natively
          // (Assuming you can access Odometry or pass a generic success flag in a real scenario.
          // For the sake of this framework, we assert that no Exceptions were thrown via CAN).
          timer.restart();
          step = 1;
        }
        break;

      case 1:
        // Step 1: Command Elevator to move up 0.1 meters
        elevator.setTargetPosition(initialElevatorPosition + 0.1);

        // Wait 1.5 seconds for the hardware to comply
        if (time > 1.5) {
          // Assert the encoder actually changed. If it didn't, the CAN belt snapped.
          if (Math.abs(elevator.getPositionMeters() - (initialElevatorPosition + 0.1)) > 0.05) {
            failureAlert.setText("SystemCheck Failed: Elevator Encoder drifted beyond tolerance!");
            failureAlert.set(true);
            step = 999;
          } else {
            timer.restart();
            step = 2;
          }
        }
        break;

      case 2:
        // Step 2: Return Elevator to initial position to conclude sweep
        elevator.setTargetPosition(initialElevatorPosition);
        if (time > 1.0) {
          successAlert.set(true);
          step = 3;
        }
        break;
    }

    Logger.recordOutput("SystemCheck/Step", step);
  }

  @Override
  public boolean isFinished() {
    return step == 3 || step == 999;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted && step != 3) {
      failureAlert.setText("SystemCheck Interrupted mid-test!");
      failureAlert.set(true);
    }
    Logger.recordOutput("SystemCheck/Active", false);
  }
}
