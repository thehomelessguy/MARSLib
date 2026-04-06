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
}
