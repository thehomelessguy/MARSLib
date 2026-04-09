package com.marslib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

/**
 * Simulates a single dynamic game piece (e.g., coral, algae) in the dyn4j physics world.
 *
 * <p>Each game piece is modeled as a circular rigid body with configurable mass, friction, and
 * bounciness. Linear and angular damping are applied so pieces naturally come to rest after being
 * bumped by the robot chassis.
 *
 * <p>When the intake subsystem collects a piece, {@link #setIntaked()} removes it from the physics
 * world and marks it as collected.
 */
public class GamePieceSim {
  private final Body gamePieceBody;
  private final String name;
  private boolean isIntaked = false;
  private double zHeightMeters = FieldConstants.GAME_PIECE_REST_HEIGHT_METERS;
  private double zVelocityMetersPerSecond = 0.0;

  /**
   * Creates a new game piece and registers it with the {@link MARSPhysicsWorld}.
   *
   * @param id A unique string identifier for this piece (used in the log key).
   * @param initialPosition The starting field-relative position of the piece center.
   */
  public GamePieceSim(String id, Translation2d initialPosition) {
    this.name = "GamePiece_" + id;
    this.gamePieceBody = new Body();

    // Model as a circle with radius from FieldConstants
    BodyFixture fixture =
        gamePieceBody.addFixture(Geometry.createCircle(FieldConstants.GAME_PIECE_RADIUS_METERS));
    // High friction and zero bounciness to mimic foam deformation/squish
    fixture.setFriction(0.8);
    fixture.setRestitution(0.0);

    // Compute density from mass and area: density = mass / (π * r²)
    double area =
        Math.PI * FieldConstants.GAME_PIECE_RADIUS_METERS * FieldConstants.GAME_PIECE_RADIUS_METERS;
    fixture.setDensity(FieldConstants.GAME_PIECE_MASS_KG / area);
    gamePieceBody.setMass(MassType.NORMAL);

    // Heavy damping so pieces dramatically decelerate and "stick" when dragged by intake
    gamePieceBody.setLinearDamping(4.5);
    gamePieceBody.setAngularDamping(5.0);

    gamePieceBody.translate(initialPosition.getX(), initialPosition.getY());

    MARSPhysicsWorld.getInstance().registerDynamicGamePiece(this);
  }

  /**
   * Returns the unique display name of this game piece.
   *
   * @return The piece name (e.g., "GamePiece_mid_1").
   */
  public String getName() {
    return name;
  }

  /**
   * Returns the underlying dyn4j body for direct physics queries.
   *
   * @return The dyn4j {@link Body} representing this piece.
   */
  public Body getBody() {
    return gamePieceBody;
  }

  /**
   * Returns the current position of the piece center on the field.
   *
   * @return A {@link Translation2d} in standard WPILib field coordinates (meters).
   */
  public Translation2d getPosition() {
    return new Translation2d(
        gamePieceBody.getTransform().getTranslationX(),
        gamePieceBody.getTransform().getTranslationY());
  }

  /**
   * Returns the full pose (position + heading) of the piece.
   *
   * @return A {@link Pose2d} in standard WPILib field coordinates.
   */
  public Pose2d getPose() {
    return new Pose2d(
        gamePieceBody.getTransform().getTranslationX(),
        gamePieceBody.getTransform().getTranslationY(),
        new Rotation2d(gamePieceBody.getTransform().getRotationAngle()));
  }

  /**
   * Returns whether this piece has been collected by the intake.
   *
   * @return {@code true} if the piece has been intaked and removed from the physics world.
   */
  public boolean isIntaked() {
    return isIntaked;
  }

  /**
   * Marks this game piece as collected and removes it from the physics world.
   *
   * <p>Called by the intake subsystem when the piece enters the collection radius.
   */
  public void setIntaked() {
    this.isIntaked = true;
    MARSPhysicsWorld.getInstance().getWorld().removeBody(gamePieceBody);
  }

  /**
   * Called automatically by MARSPhysicsWorld every simulation frame to update gravity and Z height.
   */
  public void update(double dtSeconds) {
    if (zHeightMeters > FieldConstants.GAME_PIECE_REST_HEIGHT_METERS
        || zVelocityMetersPerSecond != 0.0) {
      zVelocityMetersPerSecond -= 9.81 * dtSeconds;
      zHeightMeters += zVelocityMetersPerSecond * dtSeconds;

      // Floor collision
      if (zHeightMeters <= FieldConstants.GAME_PIECE_REST_HEIGHT_METERS) {
        zHeightMeters = FieldConstants.GAME_PIECE_REST_HEIGHT_METERS;

        Translation2d pos = getPosition();
        if (pos.getDistance(FieldConstants.BLUE_HUB_POS) < FieldConstants.HUB_SIZE_METERS
            || pos.getDistance(FieldConstants.RED_HUB_POS) < FieldConstants.HUB_SIZE_METERS) {

          // Roll out towards the center of the field
          Translation2d center =
              new Translation2d(
                  FieldConstants.FIELD_LENGTH_METERS / 2.0,
                  FieldConstants.FIELD_WIDTH_METERS / 2.0);
          Translation2d diff = center.minus(pos);
          double length = diff.getNorm();
          if (length > 0.01) {
            zVelocityMetersPerSecond = 0.0;
            gamePieceBody.setLinearVelocity(diff.getX() / length * 2.5, diff.getY() / length * 2.5);
          }
        } else {
          // Basic inelastic bounce
          zVelocityMetersPerSecond = -zVelocityMetersPerSecond * 0.3;
          // Settle when velocity drops low enough
          if (Math.abs(zVelocityMetersPerSecond) < 0.5) {
            zVelocityMetersPerSecond = 0.0;
          }
        }
      }
    }
  }

  /** Returns full pose in 3D (including simulated faked Z height). */
  public edu.wpi.first.math.geometry.Pose3d getPose3d() {
    Pose2d pose = getPose();
    return new edu.wpi.first.math.geometry.Pose3d(
        pose.getX(),
        pose.getY(),
        zHeightMeters,
        new edu.wpi.first.math.geometry.Rotation3d(0, 0, pose.getRotation().getRadians()));
  }

  /**
   * Respawns this (previously intaked) game piece into the physics world and applies initial
   * velocities.
   */
  public void launch(Translation2d origin, double vx, double vy, double vz, double initialZHeight) {
    this.isIntaked = false;
    this.zHeightMeters = initialZHeight;
    this.zVelocityMetersPerSecond = vz;

    this.gamePieceBody.getTransform().setTranslation(origin.getX(), origin.getY());
    this.gamePieceBody.setLinearVelocity(vx, vy);

    MARSPhysicsWorld.getInstance().getWorld().addBody(this.gamePieceBody);
  }
}
