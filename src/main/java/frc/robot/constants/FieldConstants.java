package frc.robot.constants;

import com.marslib.mechanisms.*;

public final class FieldConstants {
  /** Standard FRC field length (meters). */
  public static final double FIELD_LENGTH_METERS = 16.541;

  /** Standard FRC field width (meters). */
  public static final double FIELD_WIDTH_METERS = 8.069;

  /** Thickness of simulated field boundary walls (meters). */
  public static final double WALL_THICKNESS_METERS = 1.0;

  /** Approximate side length of the hub/reef structures (meters). */
  public static final double HUB_SIZE_METERS = 1.2;

  /** Height above the field surface at which game pieces rest in 3D visualization (meters). */
  public static final double GAME_PIECE_REST_HEIGHT_METERS = 0.1;

  /** Radius of a standard game piece (Fuel) for collision modeling (meters). */
  public static final double GAME_PIECE_RADIUS_METERS = 0.0635;

  /** Intake collection radius (meters) for swallowing pieces in simulation. */
  public static final double INTAKE_COLLECTION_RADIUS_METERS = 1.0;

  /** Mass of a standard game piece fuel (kg). */
  public static final double GAME_PIECE_MASS_KG = 0.05;

  /** Coulomb friction coefficient for field boundary walls and obstacles. */
  public static final double WALL_FRICTION = 0.2;

  /** Coefficient of restitution (bounciness) for field boundary walls and obstacles. */
  public static final double WALL_RESTITUTION = 0.1;

  public static final edu.wpi.first.math.geometry.Translation2d RED_HUB_POS =
      new edu.wpi.first.math.geometry.Translation2d(11.91, 4.03);
  public static final edu.wpi.first.math.geometry.Translation2d BLUE_HUB_POS =
      new edu.wpi.first.math.geometry.Translation2d(4.62, 4.03);

  public static final edu.wpi.first.math.geometry.Pose2d BLUE_NEARDEPOT_CLIMB_POSE =
      new edu.wpi.first.math.geometry.Pose2d(
          1.1, 4.86, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
  public static final edu.wpi.first.math.geometry.Pose2d BLUE_OUTPOST_CLIMB_POSE =
      new edu.wpi.first.math.geometry.Pose2d(
          1.0, 2.6, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
  public static final edu.wpi.first.math.geometry.Pose2d RED_NEARDEPOT_CLIMB_POSE =
      new edu.wpi.first.math.geometry.Pose2d(
          15.35, 3.2, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
  public static final edu.wpi.first.math.geometry.Pose2d RED_NEAROUTPOST_CLIMB_POSE =
      new edu.wpi.first.math.geometry.Pose2d(
          15.4, 5.45, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));

  private static final java.util.List<edu.wpi.first.math.geometry.Pose2d> CLIMB_POSES_BLUE =
      java.util.List.of(BLUE_NEARDEPOT_CLIMB_POSE, BLUE_OUTPOST_CLIMB_POSE);
  private static final java.util.List<edu.wpi.first.math.geometry.Pose2d> CLIMB_POSES_RED =
      java.util.List.of(RED_NEARDEPOT_CLIMB_POSE, RED_NEAROUTPOST_CLIMB_POSE);

  public static edu.wpi.first.math.geometry.Pose2d getClosestClimbingPosition(
      edu.wpi.first.math.geometry.Pose2d currentRobotPose) {
    java.util.Optional<edu.wpi.first.wpilibj.DriverStation.Alliance> alliance =
        edu.wpi.first.wpilibj.DriverStation.getAlliance();
    if (alliance.isPresent()
        && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
      return currentRobotPose.nearest(CLIMB_POSES_BLUE);
    }
    return currentRobotPose.nearest(CLIMB_POSES_RED);
  }
}
