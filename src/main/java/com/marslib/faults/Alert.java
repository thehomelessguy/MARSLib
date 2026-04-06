package com.marslib.faults;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Class for managing various alerts to be displayed on the driver station. */
public class Alert {
  private static Map<String, SendableAlerts> groups = new HashMap<>();

  private final AlertType type;
  private final String group;
  private boolean active = false;
  private double activeStartTime = 0.0;
  private String text;

  public Alert(String text, AlertType type) {
    this("Alerts", text, type);
  }

  public Alert(String group, String text, AlertType type) {
    this.group = group;
    this.text = text;
    this.type = type;

    if (!groups.containsKey(group)) {
      groups.put(group, new SendableAlerts());
    }
  }

  public void set(boolean active) {
    if (active && !this.active) {
      activeStartTime = Timer.getFPGATimestamp();
      if (type == AlertType.CRITICAL) {
        MARSFaultManager.registerCriticalFault();
      }
    } else if (!active && this.active) {
      if (type == AlertType.CRITICAL) {
        MARSFaultManager.unregisterCriticalFault();
      }
    }
    this.active = active;
    groups.get(group).updateAlert(this);
  }

  public void setText(String text) {
    this.text = text;
    if (active) {
      groups.get(group).updateAlert(this);
    }
  }

  public boolean get() {
    return active;
  }

  public static enum AlertType {
    INFO,
    WARNING,
    CRITICAL
  }

  private static class SendableAlerts {
    private final List<Alert> alerts = new ArrayList<>();

    public SendableAlerts() {}

    public void updateAlert(Alert alert) {
      if (alert.active && !alerts.contains(alert)) {
        alerts.add(alert);
      } else if (!alert.active) {
        alerts.remove(alert);
      }

      alerts.sort((a, b) -> Double.compare(a.activeStartTime, b.activeStartTime));

      List<String> infoStrings = new ArrayList<>();
      List<String> warningStrings = new ArrayList<>();
      List<String> criticalStrings = new ArrayList<>();

      for (Alert a : alerts) {
        switch (a.type) {
          case INFO:
            infoStrings.add(a.text);
            break;
          case WARNING:
            warningStrings.add(a.text);
            break;
          case CRITICAL:
            criticalStrings.add(a.text);
            break;
        }
      }

      // Use the group name in the log key so different alert groups don't overwrite each other
      String prefix = alerts.isEmpty() ? "Alerts" : alerts.get(0).group;
      Logger.recordOutput(prefix + "/Info", infoStrings.toArray(new String[0]));
      Logger.recordOutput(prefix + "/Warning", warningStrings.toArray(new String[0]));
      Logger.recordOutput(prefix + "/Critical", criticalStrings.toArray(new String[0]));
    }
  }
}
