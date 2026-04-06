package com.marslib;

import edu.wpi.first.apriltag.AprilTagFields;
import java.io.FileWriter;
import org.junit.jupiter.api.Test;

public class EnumTest {
  @Test
  public void testEnums() throws Exception {
    try (FileWriter fw = new FileWriter("enum_output2.txt")) {
      for (AprilTagFields f : AprilTagFields.values()) {
        fw.write("ENZ: " + f.name() + " -> " + f.m_resourceFile + "\n");
      }
    }
  }
}
