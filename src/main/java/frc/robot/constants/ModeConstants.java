package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Central mode configuration for robot execution context (Real hardware, Simulation, or Replay).
 *
 * <p>The mode is automatically detected from the WPILib runtime environment. To force replay mode,
 * set {@link #OVERRIDE_REPLAY} to {@code true}.
 */
public final class ModeConstants {
  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  /** Set to true to force REPLAY mode regardless of runtime environment. */
  public static final boolean OVERRIDE_REPLAY = false;

  /** Automatically detected execution mode. */
  public static final Mode CURRENT_MODE =
      OVERRIDE_REPLAY ? Mode.REPLAY : (RobotBase.isReal() ? Mode.REAL : Mode.SIM);

  /** The primary robot loop period in seconds (50Hz = 0.02s). */
  public static final double LOOP_PERIOD_SECS = 0.02;

  private ModeConstants() {}
}
