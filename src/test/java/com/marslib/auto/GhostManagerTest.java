package com.marslib.auto;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.io.File;
import java.nio.file.Files;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class GhostManagerTest {

  private File ghostFile;

  @BeforeEach
  public void setUp() {
    ghostFile = new File(frc.robot.Constants.AutoConstants.GHOST_MACRO_FILE_PATH);
    if (ghostFile.exists()) {
      ghostFile.delete();
    }
  }

  @AfterEach
  public void tearDown() {
    if (ghostFile.exists()) {
      ghostFile.delete();
    }
  }

  @Test
  public void testRecordingDaemonThreadDrainsBuffer() throws Exception {
    GhostManager manager = new GhostManager();

    // Setup dummy suppliers
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
      // Give the background thread time to not be outcompeted by the loop
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

  @Test
  public void testPlaybackInjectsValues() throws Exception {
    GhostManager manager = new GhostManager();

    // Create a mock file
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
}
