package com.marslib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;
import org.littletonrobotics.junction.Logger;

/**
 * Singleton 2D physics world managing all rigid-body interactions on the FRC field.
 *
 * <p>This class owns the dyn4j {@link World} instance and is responsible for:
 *
 * <ul>
 *   <li>Building static field boundary walls and obstacle structures.
 *   <li>Spawning and tracking dynamic game pieces.
 *   <li>Stepping the physics simulation and computing battery voltage sag.
 *   <li>Exporting all body poses to AdvantageKit for 3D visualization.
 * </ul>
 *
 * <p>The world operates in a top-down 2D plane (gravity is zero). All forces are applied explicitly
 * by subsystem-level physics solvers such as {@link SwerveChassisPhysics}.
 */
public class MARSPhysicsWorld {

  private static MARSPhysicsWorld instance;

  /**
   * Returns the singleton instance, creating it on first access.
   *
   * @return The global {@link MARSPhysicsWorld} instance.
   */
  public static MARSPhysicsWorld getInstance() {
    if (instance == null) {
      instance = new MARSPhysicsWorld();
    }
    return instance;
  }

  /** Resets the singleton instance. Intended exclusively for test isolation environments. */
  public static void resetInstance() {
    instance = null;
  }

  public World<Body> getDyn4jWorld() {
    return world;
  }

  private final World<Body> world;
  private final Map<String, Body> mechanismBodies;
  private final List<GamePieceSim> gamePieces;

  /** Accumulated current draw from all subsystems for this simulation frame (amps). */
  private double frameCurrentDrawAmps = 0.0;

  /** Most recently computed battery voltage after load modeling (volts). */
  private double simulatedVoltage = 12.0;

  private MARSPhysicsWorld() {
    instance = this;
    world = new World<>();
    // Top-down environment: gravity is zero. Forces are applied per-mechanism.
    world.setGravity(new Vector2(0, 0));
    mechanismBodies = new HashMap<>();
    gamePieces = new ArrayList<>();
    buildFieldBoundaries();
  }

  /**
   * Constructs the static field boundary walls and obstacle structures using dimensions from {@link
   * FieldConstants}.
   */
  private void buildFieldBoundaries() {
    double length = FieldConstants.FIELD_LENGTH_METERS;
    double width = FieldConstants.FIELD_WIDTH_METERS;
    double wallThickness = FieldConstants.WALL_THICKNESS_METERS;

    // 4 perimeter walls
    buildStaticRectangle(
        length / 2, width + wallThickness / 2, length + wallThickness * 2, wallThickness);
    buildStaticRectangle(length / 2, -wallThickness / 2, length + wallThickness * 2, wallThickness);
    buildStaticRectangle(-wallThickness / 2, width / 2, wallThickness, width + wallThickness * 2);
    buildStaticRectangle(
        length + wallThickness / 2, width / 2, wallThickness, width + wallThickness * 2);

    // REBUILT Season: Hub/reef structures (Hexagonal geometry)
    double hubSize = FieldConstants.HUB_SIZE_METERS;
    buildStaticHexagon(
        FieldConstants.BLUE_HUB_POS.getX(), FieldConstants.BLUE_HUB_POS.getY(), hubSize);
    buildStaticHexagon(
        FieldConstants.RED_HUB_POS.getX(), FieldConstants.RED_HUB_POS.getY(), hubSize);

    spawnInitialGamePieces();
  }

  /** Places the initial set of game pieces at their starting field positions. */
  private void spawnInitialGamePieces() {
    double midX = FieldConstants.FIELD_LENGTH_METERS / 2.0;
    double midY = FieldConstants.FIELD_WIDTH_METERS / 2.0;

    new GamePieceSim("mid_1", new edu.wpi.first.math.geometry.Translation2d(midX, midY + 1.0));
    new GamePieceSim("mid_2", new edu.wpi.first.math.geometry.Translation2d(midX, midY - 1.0));
    new GamePieceSim("mid_3", new edu.wpi.first.math.geometry.Translation2d(midX + 1.0, midY));
    new GamePieceSim("mid_4", new edu.wpi.first.math.geometry.Translation2d(midX - 1.0, midY));
  }

  private Body buildStaticRectangle(double x, double y, double w, double h) {
    Body body = new Body();
    BodyFixture fixture = body.addFixture(Geometry.createRectangle(w, h));
    fixture.setFriction(frc.robot.Constants.FieldConstants.WALL_FRICTION);
    fixture.setRestitution(frc.robot.Constants.FieldConstants.WALL_RESTITUTION);
    body.setMass(MassType.INFINITE);
    body.translate(x, y);
    world.addBody(body);
    return body;
  }

  /**
   * Creates a static hexagonal body in the physics world to simulate complex REBUILT structures.
   *
   * @param x Center X position (meters).
   * @param y Center Y position (meters).
   * @param circumradius Distance from center to any vertex (meters).
   * @return The created static {@link Body}.
   */
  private Body buildStaticHexagon(double x, double y, double circumradius) {
    Body body = new Body();
    // Create a 6-sided polygon. Polygon geometries in Dyn4j must be defined CCW.
    Vector2[] vertices = new Vector2[6];
    for (int i = 0; i < 6; i++) {
      double angle = i * Math.PI / 3.0; // 60 degrees per vertex
      vertices[i] = new Vector2(circumradius * Math.cos(angle), circumradius * Math.sin(angle));
    }

    BodyFixture fixture = body.addFixture(Geometry.createPolygon(vertices));

    // Simulate stronger friction and deadened bounciness to prevent infinite sliding off core
    // elements
    fixture.setFriction(frc.robot.Constants.FieldConstants.WALL_FRICTION * 2.5);
    fixture.setRestitution(0.01);

    body.setMass(MassType.INFINITE);
    body.translate(x, y);
    world.addBody(body);
    return body;
  }

  /**
   * Registers a dynamic game piece in the physics world.
   *
   * @param gamePiece The {@link GamePieceSim} to add.
   */
  public void registerDynamicGamePiece(GamePieceSim gamePiece) {
    gamePieces.add(gamePiece);
    world.addBody(gamePiece.getBody());
  }

  /**
   * Returns the list of all tracked game pieces (including intaked ones).
   *
   * @return An unmodifiable view would be ideal, but returns the internal list for performance.
   */
  public List<GamePieceSim> getGamePieces() {
    return gamePieces;
  }

  /**
   * Checks if any active game pieces are within collection range of the robot, marking them as
   * intaked and removing them from the physics world if so.
   *
   * @param currentRobotPose The robot's current field-relative pose.
   * @param collectionRadiusMeters The effective intake collection radius (meters).
   * @return {@code true} if at least one piece was collected this frame.
   */
  public boolean checkIntake(Pose2d currentRobotPose, double collectionRadiusMeters) {
    Iterator<GamePieceSim> iterator = gamePieces.iterator();
    boolean swallowed = false;
    while (iterator.hasNext()) {
      GamePieceSim piece = iterator.next();
      if (!piece.isIntaked()) {
        double dist = piece.getPosition().getDistance(currentRobotPose.getTranslation());
        if (dist <= collectionRadiusMeters) {
          piece.setIntaked();
          swallowed = true;
        }
      }
    }
    return swallowed;
  }

  /**
   * Returns the underlying dyn4j world for advanced operations.
   *
   * @return The {@link World} instance.
   */
  public World<Body> getWorld() {
    return world;
  }

  /**
   * Registers a named dynamic body (e.g., chassis, elevator carriage) in the physics world.
   *
   * @param name A unique identifier used as the AdvantageKit log key.
   * @param body The dyn4j {@link Body} to register.
   */
  public void registerMechanismBody(String name, Body body) {
    mechanismBodies.put(name, body);
    world.addBody(body);
  }

  /**
   * Accumulates current draw from a subsystem for this simulation frame.
   *
   * @param amps The current draw to add (amps). Should be non-negative.
   */
  public void addFrameCurrentDrawAmps(double amps) {
    frameCurrentDrawAmps += amps;
  }

  /**
   * Steps the physics world forward by one timestep, computes battery voltage sag from accumulated
   * current draw, and exports all body poses to AdvantageKit.
   *
   * @param dtSeconds The timestep duration (seconds).
   */
  public void update(double dtSeconds) {
    // Step dyn4j world
    world.step(1, dtSeconds);

    // Compute battery voltage sag from total current draw
    Logger.recordOutput("PhysicsWorld/FrameCurrentDraw_A", frameCurrentDrawAmps);
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(frameCurrentDrawAmps);
    // CRITICAL: Floor voltage at 6.0V — below this, the roboRIO would brown out and disable
    // outputs anyway. Allowing voltage to drop lower causes numerical instability in duty cycle
    // calculations (division by near-zero voltage).
    loadedVoltage = Math.max(6.0, loadedVoltage);
    simulatedVoltage = loadedVoltage;
    RoboRioSim.setVInVoltage(loadedVoltage);
    Logger.recordOutput("PhysicsWorld/ComputedVoltage", loadedVoltage);

    // Reset per-frame accumulator
    frameCurrentDrawAmps = 0.0;

    exportToAdvantageScope();
  }

  /**
   * Returns the most recently computed battery voltage after load modeling.
   *
   * @return Simulated battery voltage (volts).
   */
  public double getSimulatedVoltage() {
    return simulatedVoltage;
  }

  /**
   * Exports all mechanism body poses and game piece positions to AdvantageKit for visualization.
   *
   * <p>The dyn4j 2D coordinate system is mapped directly to the WPILib field coordinate system:
   * dyn4j X → WPILib X (field-forward), dyn4j Y → WPILib Y (field-left). Both are top-down 2D views
   * of the field plane. The Z axis is set to 0 for ground-level bodies.
   */
  private void exportToAdvantageScope() {
    // Export named mechanism bodies
    for (Map.Entry<String, Body> entry : mechanismBodies.entrySet()) {
      String mechanismName = entry.getKey();
      Body body = entry.getValue();

      double xMeters = body.getTransform().getTranslationX();
      double yMeters = body.getTransform().getTranslationY();
      double yawRads = body.getTransform().getRotationAngle();

      Pose3d pose3d = new Pose3d(xMeters, yMeters, 0.0, new Rotation3d(0.0, 0.0, yawRads));
      Logger.recordOutput("PhysicsWorld/" + mechanismName, pose3d);
    }

    // Export active game pieces
    Pose3d[] piecePoses =
        gamePieces.stream()
            .filter(piece -> !piece.isIntaked())
            .map(
                piece -> {
                  Pose2d pose2d = piece.getPose();
                  return new Pose3d(
                      pose2d.getX(),
                      pose2d.getY(),
                      FieldConstants.GAME_PIECE_REST_HEIGHT_METERS,
                      new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
                })
            .toArray(Pose3d[]::new);
    Logger.recordOutput("PhysicsWorld/GamePieces", piecePoses);
  }
}
