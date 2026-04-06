import edu.wpi.first.apriltag.AprilTagFields;

public class list_fields {
  public static void main(String[] args) {
    for (AprilTagFields f : AprilTagFields.values()) {
      System.out.println(f.name() + " -> " + f.m_resourceFile);
    }
  }
}
