package frc.robot.constants;

public final class ModeConstants {
  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public static final Mode CURRENT_MODE = Mode.SIM;
  public static final boolean OVERRIDE_SIM = false;
  public static final double LOOP_PERIOD_SECS = 0.02;

  private ModeConstants() {}
}
