package com.marslib.hmi;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * A wrapper for CommandXboxController that dynamically logs string descriptors indicating what
 * macro is mapped to what button securely to AdvantageKit and NetworkTables. This inherently allows
 * Dashboards to visually represent controller bindings safely.
 */
public class TelemetryGamepad extends CommandXboxController {

  private final String gamepadIdentity;
  private final Map<String, String> currentBindings = new HashMap<>();

  public TelemetryGamepad(int port, String gamepadIdentity) {
    super(port);
    this.gamepadIdentity = gamepadIdentity;
  }

  /**
   * Binds a command to a Trigger's `.onTrue()` scheduling method while automatically publishing the
   * mapped action name to NetworkTables/AdvantageKit.
   *
   * @param trigger The Trigger instance (e.g. this.a())
   * @param buttonName The human-readable physical button name (e.g. "A_Button")
   * @param actionName The macro the button runs (e.g. "Score Gamepiece")
   * @param cmd The Command to schedule
   * @return The Trigger to allow method chaining
   */
  public Trigger bindOnTrue(Trigger trigger, String buttonName, String actionName, Command cmd) {
    currentBindings.put(buttonName, actionName);
    Logger.recordOutput("GamepadBindings/" + gamepadIdentity + "/" + buttonName, actionName);
    return trigger.onTrue(cmd);
  }

  /**
   * Binds a command to a Trigger's `.whileTrue()` scheduling method while automatically publishing
   * the mapped action name to NetworkTables/AdvantageKit.
   *
   * @param trigger The Trigger instance (e.g. this.a())
   * @param buttonName The human-readable physical button name (e.g. "RightTrigger")
   * @param actionName The macro the button runs (e.g. "Shoot On Move")
   * @param cmd The Command to schedule
   * @return The Trigger to allow method chaining
   */
  public Trigger bindWhileTrue(Trigger trigger, String buttonName, String actionName, Command cmd) {
    currentBindings.put(buttonName, actionName);
    Logger.recordOutput("GamepadBindings/" + gamepadIdentity + "/" + buttonName, actionName);
    return trigger.whileTrue(cmd);
  }

  /**
   * Binds a command to a Trigger's `.onFalse()` scheduling method while automatically publishing
   * the mapped action name to NetworkTables/AdvantageKit.
   *
   * @param trigger The Trigger instance (e.g. this.a())
   * @param buttonName The human-readable physical button name (e.g. "A_Button")
   * @param actionName The macro the button runs (e.g. "Retract System")
   * @param cmd The Command to schedule
   * @return The Trigger to allow method chaining
   */
  public Trigger bindOnFalse(Trigger trigger, String buttonName, String actionName, Command cmd) {
    // Overwrite standard map as this is likely a dual-trigger
    currentBindings.put(buttonName, actionName + " (Release)");
    Logger.recordOutput(
        "GamepadBindings/" + gamepadIdentity + "/" + buttonName, actionName + " (Release)");
    return trigger.onFalse(cmd);
  }

  public Map<String, String> getBindingsMap() {
    return currentBindings;
  }
}
