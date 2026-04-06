package com.marslib.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
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
 * <p>Recording uses a thread-safe {@link ConcurrentLinkedQueue} buffer drained by a background
 * writer thread. This ensures zero main-loop blocking from file I/O or String formatting.
 */
public class GhostManager {

  private boolean isPlaying = false;

  private Timer timer = new Timer();

  // --- Recording (thread-safe) ---
  private volatile boolean recording = false;
  private final ConcurrentLinkedQueue<String> writeBuffer = new ConcurrentLinkedQueue<>();
  private Thread writerThread;

  // --- Playback Cache ---
  private List<GhostFrame> frames = new ArrayList<>();
  private int playIndex = 0;
  private GhostFrame currentFrame = new GhostFrame();

  /** Represents a single slice of driver inputs at a specific timestamp. */
  private static class GhostFrame {
    /** The timestamp of this frame, in seconds from start. */
    double time;
    /** Joystick axis values. */
    double leftY, leftX, rightX;
    /** Button states. */
    boolean a, b, x, y, lb, rb, up, down, left, right;
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
   * Starts a daemon thread that continuously drains the write buffer to the given {@link
   * PrintWriter}. The thread sleeps briefly between drain cycles to avoid busy-spinning while
   * keeping latency well below one robot loop period.
   */
  private void startWriterThread(PrintWriter pw) {
    writerThread =
        new Thread(
            () -> {
              while (recording || !writeBuffer.isEmpty()) {
                String line;
                while ((line = writeBuffer.poll()) != null) {
                  pw.println(line);
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
              pw.close();
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
      BooleanSupplier right) {
    return new Command() {
      @Override
      public void initialize() {
        try {
          File file = new File(frc.robot.Constants.AutoConstants.GHOST_MACRO_FILE_PATH);
          PrintWriter pw = new PrintWriter(new FileWriter(file));
          // Write CSV header synchronously (single write, negligible cost)
          pw.println("time,ly,lx,rx,a,b,x,y,lb,rb,up,down,left,right");

          recording = true;
          writeBuffer.clear();
          startWriterThread(pw);
          timer.restart();

          Logger.recordOutput("Ghost/IsRecording", true);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }

      @Override
      public void execute() {
        if (recording) {
          // Enqueue to the lock-free buffer — zero blocking on the main thread
          writeBuffer.offer(
              String.format(
                  "%.3f,%.3f,%.3f,%.3f,%b,%b,%b,%b,%b,%b,%b,%b,%b,%b",
                  timer.get(),
                  leftY.getAsDouble(),
                  leftX.getAsDouble(),
                  rightX.getAsDouble(),
                  a.getAsBoolean(),
                  b.getAsBoolean(),
                  x.getAsBoolean(),
                  y.getAsBoolean(),
                  lb.getAsBoolean(),
                  rb.getAsBoolean(),
                  up.getAsBoolean(),
                  down.getAsBoolean(),
                  left.getAsBoolean(),
                  right.getAsBoolean()));
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
  public Command getPlaybackCommand() {
    return new Command() {
      @Override
      public void initialize() {
        frames.clear();
        playIndex = 0;
        try (BufferedReader br =
            new BufferedReader(
                new FileReader(frc.robot.Constants.AutoConstants.GHOST_MACRO_FILE_PATH))) {
          String line = br.readLine(); // skip header
          while ((line = br.readLine()) != null) {
            String[] v = line.split(",");
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
          }
        } catch (Exception e) {
          System.err.println("GHOST FILE NOT FOUND: Cannot play back.");
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
