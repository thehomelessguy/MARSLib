package com.marslib.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.HashMap;
import java.util.Map;
import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;
import org.littletonrobotics.junction.Logger;

public class MARSPhysicsWorld {

  private static MARSPhysicsWorld instance;

  public static MARSPhysicsWorld getInstance() {
    if (instance == null) {
      instance = new MARSPhysicsWorld();
    }
    return instance;
  }

  private final World<Body> world;
  private final Map<String, Body> mechanismBodies;

  private double frameCurrentDrawAmps = 0.0;

  private double simulatedVoltageDrop = 12.0;

  private MARSPhysicsWorld() {
    world = new World<>();
    world.setGravity(World.EARTH_GRAVITY); // dyn4j defaults to standard Earth gravity (0, -9.81)
    mechanismBodies = new HashMap<>();
  }

  public World<Body> getWorld() {
    return world;
  }

  public void registerMechanismBody(String name, Body body) {
    mechanismBodies.put(name, body);
    world.addBody(body);
  }

  public void addFrameCurrentDrawAmps(double amps) {
    frameCurrentDrawAmps += amps;
  }

  public void update(double dtSeconds) {
    // Step Dyn4j 2D World
    world.step(1, dtSeconds);

    // Calculate System Brownout Resistance
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(frameCurrentDrawAmps);
    simulatedVoltageDrop = loadedVoltage;
    RoboRioSim.setVInVoltage(loadedVoltage);

    // Reset accumulator
    frameCurrentDrawAmps = 0.0;

    exportToAdvantageScope();
  }

  public double getSimulatedVoltage() {
    return simulatedVoltageDrop;
  }

  private void exportToAdvantageScope() {
    // Iterate named bodies and export directly to AdvantageKit 3D Render space
    for (Map.Entry<String, Body> entry : mechanismBodies.entrySet()) {
      String mechanismName = entry.getKey();
      Body body = entry.getValue();

      // Dyn4j is 2D (X, Y plane). WPILib Pose3D assigns:
      // X = Forward/Back
      // Y = Left/Right
      // Z = Up/Down
      // Therefore, Dyn4j X -> WPILib X
      // Dyn4j Y -> WPILib Z
      // Dyn4j Rotation -> WPILib Pitch (Rotation about Y axis)

      double xMeters = body.getTransform().getTranslationX();
      double zMeters = body.getTransform().getTranslationY();
      double pitchRads = body.getTransform().getRotationAngle();

      Pose3d pose3d =
          new Pose3d(
              xMeters,
              0.0,
              zMeters,
              new Rotation3d(0.0, -pitchRads, 0.0) // Invert pitch simply to match coordinate rule
              );

      Logger.recordOutput("PhysicsWorld/" + mechanismName, pose3d);
    }
  }
}
