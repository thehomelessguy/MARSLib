// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double LOOP_PERIOD_SECS = 0.01;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final String CANBUS = "rio";

    // Front Left
    public static final int FL_DRIVE_ID = 1;
    public static final int FL_TURN_ID = 2;
    // Front Right
    public static final int FR_DRIVE_ID = 3;
    public static final int FR_TURN_ID = 4;
    // Back Left
    public static final int BL_DRIVE_ID = 5;
    public static final int BL_TURN_ID = 6;
    // Back Right
    public static final int BR_DRIVE_ID = 7;
    public static final int BR_TURN_ID = 8;

    public static final int PIGEON2_ID = 9;
  }

  public static final class ElevatorConstants {
    public static final int MOTOR_ID = 20;
    public static final String CANBUS = "rio";
    public static final double GEAR_RATIO = 10.0;
    public static final double SPOOL_DIAMETER_METERS = 0.05;
    public static final double SIM_MASS_KG = 0.01;
    public static final boolean INVERTED = false;

    // Load Shedding Bound Limits
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double CRITICAL_VOLTAGE = 7.0;
    public static final double MAX_CURRENT_AMPS = 60.0;
    public static final double MIN_CURRENT_AMPS = 20.0;
  }

  public static final class ArmConstants {
    public static final int MOTOR_ID = 21;
    public static final String CANBUS = "rio";
    public static final double GEAR_RATIO = 50.0;
    public static final double SIM_MOI = 1.0;
    public static final double SIM_LENGTH = 0.05;
    public static final boolean INVERTED = false;

    // Load Shedding Bound Limits
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double CRITICAL_VOLTAGE = 7.0;
    public static final double MAX_CURRENT_AMPS = 60.0;
    public static final double MIN_CURRENT_AMPS = 20.0;
  }

  public static final class LEDConstants {
    public static final int PWM_PORT = 0;
    public static final int LENGTH = 60;
  }

  public static final class PowerConstants {
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double WARNING_VOLTAGE = 8.0;
    public static final double CRITICAL_VOLTAGE = 7.0;
  }

  public static final class AutoConstants {
    public static final double MAX_VELOCITY_MPS = 3.0;
    public static final double MAX_ACCELERATION_MPS2 = 2.0;
    public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 = Math.PI / 2.0;
  }
}
