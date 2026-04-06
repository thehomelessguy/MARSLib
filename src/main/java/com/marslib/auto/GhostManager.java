package com.marslib.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Next-Gen Ghost Mode Automator. Handles both serializing human driver practice matches to the
 * RoboRIO flash storage via blazing-fast CSV structures, and replaying those EXACT physics loops
 * back into the teleop sequence for autonomous runs.
 */
public class GhostManager {

  private static final String FILE_PATH = "/home/lvuser/ghost_macro.csv";

  private boolean isPlaying = false;

  private PrintWriter writer;
  private Timer timer = new Timer();

  // Playback Cache
  private List<GhostFrame> frames = new ArrayList<>();
  private int playIndex = 0;
  private GhostFrame currentFrame = new GhostFrame();

  private static class GhostFrame {
    double time;
    double leftY, leftX, rightX;
    boolean a, b, x, y, lb, rb, up, down, left, right;
  }

  // Injectable Suppliers for RobotContainer
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
          File file = new File(FILE_PATH);
          writer = new PrintWriter(new FileWriter(file));
          // Fast header
          writer.println("time,ly,lx,rx,a,b,x,y,lb,rb,up,down,left,right");
          timer.restart();

          SmartDashboard.putBoolean("Ghost/IsRecording", true);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }

      @Override
      public void execute() {
        if (writer != null) {
          writer.println(
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
        if (writer != null) {
          writer.close();
        }

        SmartDashboard.putBoolean("Ghost/IsRecording", false);
      }
    };
  }

  /** Command used in Autonomous to deserialize the CSV and push it into the subsystems. */
  public Command getPlaybackCommand() {
    return new Command() {
      @Override
      public void initialize() {
        frames.clear();
        playIndex = 0;
        try (BufferedReader br = new BufferedReader(new FileReader(FILE_PATH))) {
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
        SmartDashboard.putBoolean("Ghost/IsPlaying", true);
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
        SmartDashboard.putBoolean("Ghost/IsPlaying", false);
      }
    };
  }
}
