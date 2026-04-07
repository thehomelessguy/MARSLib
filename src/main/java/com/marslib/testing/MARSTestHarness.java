package com.marslib.testing;

import com.marslib.faults.Alert;
import com.marslib.faults.MARSFaultManager;
import com.marslib.simulation.MARSPhysicsWorld;
import com.marslib.vision.AprilTagVisionIOSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Centralized test setup utility that resets ALL MARSLib static singletons in one call.
 *
 * <p>MARSLib uses 5 static singletons that accumulate state across test runs. Forgetting to reset
 * ANY of them causes cross-test contamination: stacked physics bodies, stale alerts, leaked
 * commands. This class eliminates that failure mode by providing a single reset method.
 *
 * <p><b>Usage:</b>
 *
 * <pre>{@code
 * @BeforeEach
 * public void setUp() {
 *     MARSTestHarness.reset();
 *     // ... construct your subsystems here
 * }
 * }</pre>
 */
public final class MARSTestHarness {

  private MARSTestHarness() {} // Utility class — no instances

  /**
   * Resets all static singletons to a clean state for test isolation. This MUST be called in every
   * test class's {@code @BeforeEach} method before constructing any subsystems.
   *
   * <p>Resets the following:
   *
   * <ol>
   *   <li>{@code HAL} — Initializes the WPILib Hardware Abstraction Layer for simulation
   *   <li>{@code CommandScheduler} — Cancels all commands and unregisters all subsystems
   *   <li>{@code MARSPhysicsWorld} — Destroys the dyn4j world and all physics bodies
   *   <li>{@code AprilTagVisionIOSim} — Destroys the shared VisionSystemSim
   *   <li>{@code Alert} — Clears all AdvantageScope alert groups
   *   <li>{@code MARSFaultManager} — Clears all registered faults
   *   <li>{@code DriverStationSim} — Sets Blue1 alliance, enables the robot, and beats heartbeat
   * </ol>
   */
  public static void reset() {
    // 1. HAL initialization
    HAL.initialize(500, 0);

    // 2. CommandScheduler — must cancel before unregister to avoid stale command references
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();

    // 3. Physics world — destroys all dyn4j bodies, boundaries, and game pieces
    MARSPhysicsWorld.resetInstance();

    // 4. Vision sim — destroys shared VisionSystemSim and field layout cache
    AprilTagVisionIOSim.resetSimulation();

    // 5. Alert system — clears static alert group map
    Alert.resetAll();

    // 6. Fault manager — clears critical fault state
    MARSFaultManager.clear();

    // 7. DriverStation — configure sim heartbeat
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /**
   * Tears down test state after a test completes. Call this in {@code @AfterEach} to prevent
   * subsystem references from leaking between tests.
   */
  public static void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }
}
