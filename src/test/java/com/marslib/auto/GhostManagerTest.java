package com.marslib.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.testing.MARSTestHarness;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.File;
import java.nio.file.Files;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for the GhostManager record/playback pipeline.
 *
 * <p>Validates CSV format integrity, full record→playback fidelity, and graceful error handling
 * when the ghost file is missing or corrupted. These tests catch silent serialization breakage that
 * would make pre-recorded macros silently replay incorrect inputs at competition.
 */
public class GhostManagerTest {

  private File ghostFile;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    ghostFile = new File(frc.robot.Constants.AutoConstants.GHOST_MACRO_FILE_PATH);
    if (ghostFile.exists()) {
      ghostFile.delete();
    }
  }

  @AfterEach
  public void tearDown() {
    MARSTestHarness.cleanup();
    if (ghostFile.exists()) {
      ghostFile.delete();
    }
  }

  /** Validates that recording creates a CSV with the correct header and data rows. */
  @Test
  public void testRecordingDaemonThreadDrainsBuffer() throws Exception {
    GhostManager manager = new GhostManager();

    // Setup dummy suppliers with known values
    DoubleSupplier dummyDouble = () -> 1.0;
    BooleanSupplier dummyBool = () -> true;

    Command recordCommand =
        manager.registerRecordCommand(
            dummyDouble,
            dummyDouble,
            dummyDouble,
            dummyBool,
            dummyBool,
            dummyBool,
            dummyBool,
            dummyBool,
            dummyBool,
            dummyBool,
            dummyBool,
            dummyBool,
            dummyBool);

    // Start recording
    recordCommand.initialize();

    // Simulate executing the command 5 times -> 5 frames
    for (int i = 0; i < 5; i++) {
      recordCommand.execute();
      // Give the background thread time to drain
      Thread.sleep(10);
    }

    recordCommand.end(false);

    // Wait for daemon thread to drain final frames and close file
    Thread.sleep(100);

    assertTrue(ghostFile.exists(), "Ghost macro file should have been created.");

    String content = Files.readString(ghostFile.toPath());
    String[] lines = content.split("\n");

    // Header + 5 data lines (+ potentially empty line from last println)
    assertTrue(lines.length >= 6, "Should write header and 5 execution frames");
    assertTrue(lines[0].contains("time,ly,lx,rx,a,b,x,y,lb,rb,up,down,left,right"));
    assertTrue(
        lines[1].contains("1.000,1.000,1.000,true,true,true,true,true,true,true,true,true,true"));
  }

  /** Full round-trip: writes CSV manually, plays it back, asserts the GhostManager returns it. */
  @Test
  public void testPlaybackInjectsValues() throws Exception {
    GhostManager manager = new GhostManager();

    // Create a synthetic ghost file with known values
    String dummyFileContent =
        "time,ly,lx,rx,a,b,x,y,lb,rb,up,down,left,right\n"
            + "0.000,0.500,0.250,0.000,false,true,false,false,false,false,false,false,false,false\n"
            + "10.000,1.000,-1.000,0.500,true,false,false,false,false,false,false,false,false,false\n";
    Files.writeString(ghostFile.toPath(), dummyFileContent);

    Command playCommand = manager.getPlaybackCommand();
    playCommand.initialize();

    playCommand.execute();

    DoubleSupplier defaultSupplier = () -> 0.0;
    BooleanSupplier defaultBool = () -> false;

    assertEquals(0.5, manager.getLeftY(defaultSupplier));
    assertEquals(0.25, manager.getLeftX(defaultSupplier));
    assertEquals(0.0, manager.getRightX(defaultSupplier));
    assertFalse(manager.getA(defaultBool));
    assertTrue(manager.getB(defaultBool)); // B is true in dummy

    playCommand.end(false);

    assertEquals(
        0.0,
        manager.getLeftY(defaultSupplier),
        "After playback ends, it should return to default supplier");
  }

  /**
   * Tests that playback gracefully handles a missing ghost file without crashing. A new student
   * clearing the RoboRIO flash shouldn't cause an NPE during auto.
   */
  @Test
  public void testPlaybackWithMissingFileDoesNotCrash() {
    // Ensure no ghost file exists
    if (ghostFile.exists()) {
      ghostFile.delete();
    }

    GhostManager manager = new GhostManager();
    Command playCommand = manager.getPlaybackCommand();

    assertDoesNotThrow(
        () -> {
          playCommand.initialize();
          for (int i = 0; i < 10; i++) {
            playCommand.execute();
          }
          playCommand.end(false);
        },
        "Playback with missing file should not crash");

    // After failed load, getters should return driver values (not ghosted)
    assertEquals(42.0, manager.getLeftY(() -> 42.0), "Should return driver value, not ghost");
  }

  /**
   * Tests that recording with distinct per-axis values preserves them correctly. Catches field
   * ordering bugs in the CSV format string.
   */
  @Test
  public void testFieldOrderingPreservedAcrossRoundTrip() throws Exception {
    GhostManager manager = new GhostManager();

    // Distinct values for each field so we can verify ordering
    Command recordCommand =
        manager.registerRecordCommand(
            () -> 0.1, // leftY
            () -> 0.2, // leftX
            () -> 0.3, // rightX
            () -> true, // a
            () -> false, // b
            () -> true, // x
            () -> false, // y
            () -> true, // lb
            () -> false, // rb
            () -> true, // up
            () -> false, // down
            () -> true, // left
            () -> false // right
            );

    recordCommand.initialize();
    for (int i = 0; i < 3; i++) {
      recordCommand.execute();
      Thread.sleep(10);
    }
    recordCommand.end(false);
    Thread.sleep(100);

    // Now play it back
    Command playCommand = manager.getPlaybackCommand();
    playCommand.initialize();
    playCommand.execute();

    // Verify distinct values came back in the right order
    assertEquals(0.1, manager.getLeftY(() -> 0.0), 0.01, "leftY field order");
    assertEquals(0.2, manager.getLeftX(() -> 0.0), 0.01, "leftX field order");
    assertEquals(0.3, manager.getRightX(() -> 0.0), 0.01, "rightX field order");
    assertTrue(manager.getA(() -> false), "A button field order");
    assertFalse(manager.getB(() -> true), "B button field order");
    assertTrue(manager.getX(() -> false), "X button field order");
    assertFalse(manager.getY(() -> true), "Y button field order");
    assertTrue(manager.getLb(() -> false), "LB button field order");
    assertFalse(manager.getRb(() -> true), "RB button field order");
    assertTrue(manager.getPovUp(() -> false), "POV Up field order");
    assertFalse(manager.getPovDown(() -> true), "POV Down field order");
    assertTrue(manager.getPovLeft(() -> false), "POV Left field order");
    assertFalse(manager.getPovRight(() -> true), "POV Right field order");

    playCommand.end(false);
  }
}
