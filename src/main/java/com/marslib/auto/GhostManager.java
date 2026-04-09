package com.marslib.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Next-Gen Ghost Mode Automator. Handles both serializing human driver practice matches to the
 * RoboRIO flash storage via blazing-fast CSV structures, and replaying those EXACT physics loops
 * back into the teleop sequence for autonomous runs.
 *
 * <p><b>Concurrency &amp; Hardware Safety</b> Re-writing to the physical flash block storage of the
 * RoboRIO takes milliseconds, which would catastrophically lag the primary 50Hz robotic control
 * loop. To bypass this, this class implements a thread-safe {@link ConcurrentLinkedQueue} buffer.
 * The main robot loop exclusively pushes memory-light Strings to the queue without locking.
 *
 * <p>A separate background daemon thread wakes up every ~5ms to drain the queue chunks to disk,
 * guaranteeing zero main-loop blocking from file I/O formatting.
 */
public class GhostManager {

  private volatile boolean isPlaying = false;

  private Timer timer = new Timer();

  // --- Recording (thread-safe) ---
  private volatile boolean recording = false;
  private final ConcurrentLinkedQueue<String> writeBuffer = new ConcurrentLinkedQueue<>();
  private Thread writerThread;

  @SuppressWarnings("PMD.AvoidStringBufferField")
  private final StringBuilder rowBuilder = new StringBuilder(128);

  // --- Playback Cache ---
  private List<GhostFrame> frames = new ArrayList<>();
  private int playIndex = 0;
  private GhostFrame currentFrame = new GhostFrame();

  /** Represents a single slice of driver inputs at a specific timestamp. */
  private static class GhostFrame {
    /** The timestamp of this frame, in seconds from start. */
    double time;
    /** Joystick axis values. */
    double leftY;

    double leftX;
    double rightX;
    /** Button states. */
    boolean a;

    boolean b;
    boolean x;
    boolean y;
    boolean lb;
    boolean rb;
    boolean up;
    boolean down;
    boolean left;
    boolean right;
  }

  // ---------------------------------------------------------------------------
  // Injectable Suppliers for RobotContainer
  // ---------------------------------------------------------------------------

  public double getLeftY(DoubleSupplier driverAxis) {
    return isPlaying ? currentFrame.leftY : driverAxis.getAsDouble();
  }

  public double getLeftX(DoubleSupplier driverAxis) {
    return isPlaying ? currentFrame.leftX : driverAxis.getAsDouble();
  }

  public double getRightX(DoubleSupplier driverAxis) {
    return isPlaying ? currentFrame.rightX : driverAxis.getAsDouble();
  }

  public boolean getA(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.a : driverBtn.getAsBoolean();
  }

  public boolean getB(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.b : driverBtn.getAsBoolean();
  }

  public boolean getX(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.x : driverBtn.getAsBoolean();
  }

  public boolean getY(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.y : driverBtn.getAsBoolean();
  }

  public boolean getLb(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.lb : driverBtn.getAsBoolean();
  }

  public boolean getRb(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.rb : driverBtn.getAsBoolean();
  }

  public boolean getPovUp(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.up : driverBtn.getAsBoolean();
  }

  public boolean getPovDown(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.down : driverBtn.getAsBoolean();
  }

  public boolean getPovLeft(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.left : driverBtn.getAsBoolean();
  }

  public boolean getPovRight(BooleanSupplier driverBtn) {
    return isPlaying ? currentFrame.right : driverBtn.getAsBoolean();
  }

  // ---------------------------------------------------------------------------
  // Background writer thread — drains the ConcurrentLinkedQueue to disk
  // ---------------------------------------------------------------------------

  /**
   * Starts a daemon thread that continuously drains the write buffer to the given {@link File}. The
   * thread sleeps briefly between drain cycles to avoid busy-spinning while keeping latency well
   * below one robot loop period.
   */
  private void startWriterThread(File file) {
    writerThread =
        new Thread(
            () -> {
              try (PrintWriter pw =
                  new PrintWriter(
                      java.nio.file.Files.newBufferedWriter(
                          file.toPath(), java.nio.charset.StandardCharsets.UTF_8))) {
                pw.println("time,ly,lx,rx,a,b,x,y,lb,rb,up,down,left,right");
                while (recording || !writeBuffer.isEmpty()) {
                  String line = writeBuffer.poll();
                  while (line != null) {
                    pw.println(line);
                    line = writeBuffer.poll();
                  }
                  // Flush after each drain cycle to survive unexpected power-offs
                  pw.flush();
                  try {
                    Thread.sleep(5); // ~200 Hz drain rate — far faster than 50 Hz production rate
                  } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                  }
                }
              } catch (Exception e) {
                DriverStation.reportError(
                    "GhostWriter failed: " + e.getMessage(), e.getStackTrace());
              }
            },
            "GhostWriter");
    writerThread.setDaemon(true);
    writerThread.start();
  }

  // ---------------------------------------------------------------------------
  // Recording Command
  // ---------------------------------------------------------------------------

  /** Command to hook to the Driver controller to stream recording to flash disk. */
  public Command registerRecordCommand(
      DoubleSupplier leftY,
      DoubleSupplier leftX,
      DoubleSupplier rightX,
      BooleanSupplier a,
      BooleanSupplier b,
      BooleanSupplier x,
      BooleanSupplier y,
      BooleanSupplier lb,
      BooleanSupplier rb,
      BooleanSupplier up,
      BooleanSupplier down,
      BooleanSupplier left,
      BooleanSupplier right,
      Subsystem... requirements) {
    return new Command() {
      {
        addRequirements(requirements);
      }

      @Override
      public void initialize() {
        try {
          // Safely terminate any existing writer thread from a previous recording
          if (writerThread != null && writerThread.isAlive()) {
            recording = false; // Signal the old thread to drain and finish
            writerThread.interrupt();
            try {
              writerThread.join(100); // Wait up to 100ms for clean shutdown
            } catch (InterruptedException e) {
              Thread.currentThread().interrupt();
            }
          }

          File file = new File(frc.robot.constants.AutoConstants.GHOST_MACRO_FILE_PATH);

          recording = true;
          writeBuffer.clear();
          startWriterThread(file);
          timer.restart();

          Logger.recordOutput("Ghost/IsRecording", true);
        } catch (Exception e) {
          DriverStation.reportError(
              "Failed to initialize Ghost recording: " + e.getMessage(), e.getStackTrace());
        }
      }

      @Override
      public void execute() {
        if (recording) {
          rowBuilder.setLength(0);
          rowBuilder
              .append(Math.round(timer.get() * 1000.0) / 1000.0)
              .append(',')
              .append(Math.round(leftY.getAsDouble() * 1000.0) / 1000.0)
              .append(',')
              .append(Math.round(leftX.getAsDouble() * 1000.0) / 1000.0)
              .append(',')
              .append(Math.round(rightX.getAsDouble() * 1000.0) / 1000.0)
              .append(',')
              .append(a.getAsBoolean())
              .append(',')
              .append(b.getAsBoolean())
              .append(',')
              .append(x.getAsBoolean())
              .append(',')
              .append(y.getAsBoolean())
              .append(',')
              .append(lb.getAsBoolean())
              .append(',')
              .append(rb.getAsBoolean())
              .append(',')
              .append(up.getAsBoolean())
              .append(',')
              .append(down.getAsBoolean())
              .append(',')
              .append(left.getAsBoolean())
              .append(',')
              .append(right.getAsBoolean());

          // Enqueue to the lock-free buffer — zero blocking on the main thread
          writeBuffer.offer(rowBuilder.toString());
        }
      }

      @Override
      public void end(boolean interrupted) {
        recording = false;
        // The writer thread will drain remaining buffer entries and close the PrintWriter
        Logger.recordOutput("Ghost/IsRecording", false);
      }
    };
  }

  // ---------------------------------------------------------------------------
  // Playback Command
  // ---------------------------------------------------------------------------

  /** Command used in Autonomous to deserialize the CSV and push it into the subsystems. */
  public Command getPlaybackCommand(Subsystem... requirements) {
    return new Command() {
      {
        addRequirements(requirements);
      }

      @Override
      public void initialize() {
        frames.clear();
        playIndex = 0;
        try (BufferedReader br =
            java.nio.file.Files.newBufferedReader(
                java.nio.file.Paths.get(frc.robot.constants.AutoConstants.GHOST_MACRO_FILE_PATH),
                java.nio.charset.StandardCharsets.UTF_8)) {
          br.readLine(); // skip header
          String line = br.readLine();
          while (line != null) {
            @SuppressWarnings("StringSplitter")
            String[] v = line.split(",", -1);
            GhostFrame frame = new GhostFrame();
            frame.time = Double.parseDouble(v[0]);
            frame.leftY = Double.parseDouble(v[1]);
            frame.leftX = Double.parseDouble(v[2]);
            frame.rightX = Double.parseDouble(v[3]);
            frame.a = Boolean.parseBoolean(v[4]);
            frame.b = Boolean.parseBoolean(v[5]);
            frame.x = Boolean.parseBoolean(v[6]);
            frame.y = Boolean.parseBoolean(v[7]);
            frame.lb = Boolean.parseBoolean(v[8]);
            frame.rb = Boolean.parseBoolean(v[9]);
            frame.up = Boolean.parseBoolean(v[10]);
            frame.down = Boolean.parseBoolean(v[11]);
            frame.left = Boolean.parseBoolean(v[12]);
            frame.right = Boolean.parseBoolean(v[13]);
            frames.add(frame);
            line = br.readLine();
          }
        } catch (Exception e) {
          DriverStation.reportError("GHOST FILE NOT FOUND: Cannot play back.", false);
          return; // failed
        }

        isPlaying = true;
        Logger.recordOutput("Ghost/IsPlaying", true);
        timer.restart();
      }

      @Override
      public void execute() {
        if (!isPlaying || frames.isEmpty()) return;

        double t = timer.get();
        // Fast-forward array index to current playback time
        while (playIndex < frames.size() - 1 && frames.get(playIndex + 1).time <= t) {
          playIndex++;
        }
        currentFrame = frames.get(playIndex);
      }

      @Override
      public boolean isFinished() {
        return playIndex >= frames.size() - 1;
      }

      @Override
      public void end(boolean interrupted) {
        isPlaying = false;
        Logger.recordOutput("Ghost/IsPlaying", false);
      }
    };
  }
}
