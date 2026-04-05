package frc.robot;

import com.marslib.hmi.OperatorInterface;
import com.marslib.mechanisms.LinearMechanismIOSim;
import com.marslib.mechanisms.LinearMechanismIOTalonFX;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.mechanisms.MARSSuperstructure;
import com.marslib.mechanisms.RotaryMechanismIOSim;
import com.marslib.mechanisms.RotaryMechanismIOTalonFX;
import com.marslib.power.MARSPowerManager;
import com.marslib.power.PowerIOReal;
import com.marslib.power.PowerIOSim;
import com.marslib.swerve.SwerveDrive;
import com.marslib.swerve.SwerveModule;
import com.marslib.swerve.SwerveModuleIOSim;
import com.marslib.swerve.SwerveModuleIOTalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final MARSPowerManager powerManager;
  private final OperatorInterface operatorInterface;

  private final SwerveDrive swerveDrive;
  private final MARSElevator elevator;
  private final MARSArm arm;
  private final MARSSuperstructure superstructure;

  public RobotContainer() {
    // 1. Dependency Injection based on Current Mode
    if (Constants.currentMode == Constants.Mode.SIM) {
      powerManager = new MARSPowerManager(new PowerIOSim());

      // Initialize with physics bounds
      swerveDrive =
          new SwerveDrive(
              new SwerveModule[] {
                new SwerveModule(0, new SwerveModuleIOSim()),
                new SwerveModule(1, new SwerveModuleIOSim()),
                new SwerveModule(2, new SwerveModuleIOSim()),
                new SwerveModule(3, new SwerveModuleIOSim())
              },
              powerManager);

      elevator =
          new MARSElevator(new LinearMechanismIOSim("Elevator", 10.0, 0.05, 0.01), powerManager);
      arm = new MARSArm(new RotaryMechanismIOSim("Arm", 50.0, 1.0, 0.05), powerManager);
    } else {
      powerManager = new MARSPowerManager(new PowerIOReal());

      // Initialize with Real Hardware
      swerveDrive =
          new SwerveDrive(
              new SwerveModule[] {
                new SwerveModule(0, new SwerveModuleIOTalonFX(1, 2, "rio")),
                new SwerveModule(1, new SwerveModuleIOTalonFX(3, 4, "rio")),
                new SwerveModule(2, new SwerveModuleIOTalonFX(5, 6, "rio")),
                new SwerveModule(3, new SwerveModuleIOTalonFX(7, 8, "rio"))
              },
              powerManager);

      elevator =
          new MARSElevator(
              new LinearMechanismIOTalonFX(20, "rio", 10.0, 0.05, false), powerManager);
      arm = new MARSArm(new RotaryMechanismIOTalonFX(21, "rio", 50.0, false), powerManager);
    }

    operatorInterface = new OperatorInterface(0, powerManager);
    superstructure = new MARSSuperstructure(elevator, arm);

    // 2. Configure Default Commands
    configureDefaultCommands();

    // 3. Configure Button Bindings
    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    CommandXboxController controller = operatorInterface.getController();

    // Drive with left thumbstick (translation) and right thumbstick X (rotation)
    swerveDrive.setDefaultCommand(
        swerveDrive.run(
            () -> {
              double linearMag = 4.5;
              double angularMag = Math.PI * 2;

              double xVelocity = -MathUtil.applyDeadband(controller.getLeftY(), 0.1) * linearMag;
              double yVelocity = -MathUtil.applyDeadband(controller.getLeftX(), 0.1) * linearMag;
              double omega = -MathUtil.applyDeadband(controller.getRightX(), 0.1) * angularMag;

              swerveDrive.runVelocity(new ChassisSpeeds(xVelocity, yVelocity, omega));
            }));
  }

  private void configureButtonBindings() {
    CommandXboxController controller = operatorInterface.getController();

    // DPad Up -> Score High State
    controller
        .povUp()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.SCORE_HIGH));

    // DPad Down -> Stow State
    controller
        .povDown()
        .onTrue(superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.STOWED));

    // DPad Right -> Intake State
    controller
        .povRight()
        .onTrue(
            superstructure.setAbsoluteState(MARSSuperstructure.SuperstructureState.INTAKE_FLOOR));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
