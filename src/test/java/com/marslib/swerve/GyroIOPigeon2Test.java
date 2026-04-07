package com.marslib.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class GyroIOPigeon2Test {

  private MockedConstruction<Pigeon2> pigeonMockedConstruction;
  private MockedStatic<PhoenixOdometryThread> mockThreadStatic;

  @BeforeEach
  public void setUp() {
    pigeonMockedConstruction =
        mockConstruction(
            Pigeon2.class,
            (mock, context) -> {
              StatusSignal<Angle> mockYaw = mock(StatusSignal.class);
              StatusSignal<AngularVelocity> mockYawVel = mock(StatusSignal.class);

              when(mock.getYaw()).thenReturn(mockYaw);
              when(mock.getAngularVelocityZWorld()).thenReturn(mockYawVel);
            });

    // Mock Singleton thread to not crash the JNI inside the CTRE signals when registering
    PhoenixOdometryThread fakeThread = mock(PhoenixOdometryThread.class);
    when(fakeThread.getGyroYawData())
        .thenReturn(new double[] {90.0, 91.0}); // Fake buffered odometry frames
    mockThreadStatic = mockStatic(PhoenixOdometryThread.class);
    mockThreadStatic.when(PhoenixOdometryThread::getInstance).thenReturn(fakeThread);
  }

  @AfterEach
  public void tearDown() {
    pigeonMockedConstruction.close();
    mockThreadStatic.close();
  }

  @Test
  @SuppressWarnings("unchecked")
  public void testPigeon2InitializesAndReadsCorrectly() {
    GyroIOPigeon2 gyro = new GyroIOPigeon2(0, "rio");

    assertEquals(1, pigeonMockedConstruction.constructed().size());
    Pigeon2 mockPigeon = pigeonMockedConstruction.constructed().get(0);

    StatusSignal<Angle> yawMock = mockPigeon.getYaw();
    StatusSignal<AngularVelocity> velMock = mockPigeon.getAngularVelocityZWorld();

    when(yawMock.getValueAsDouble()).thenReturn(45.0);
    when(yawMock.getStatus()).thenReturn(StatusCode.OK);

    when(velMock.getValueAsDouble()).thenReturn(10.0);

    // Provide a mocked refreshAll that does nothing since it's static
    try (MockedStatic<BaseStatusSignal> bssMock = mockStatic(BaseStatusSignal.class)) {
      GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
      gyro.updateInputs(inputs);

      assertTrue(inputs.connected, "Gyro should report connected optimally.");

      // 45 degrees should correctly unpack to PI / 4 radians
      assertEquals(Math.PI / 4.0, inputs.yawPositionRad, 0.001);

      // 10 degrees/s to rad/s
      assertEquals(10.0 * (Math.PI / 180.0), inputs.yawVelocityRadPerSec, 0.001);

      // Verify the odometry thread buffer drained 90 and 91 degrees into radians correctly
      assertEquals(2, inputs.odometryYawPositions.length);
      assertEquals(90.0 * (Math.PI / 180.0), inputs.odometryYawPositions[0], 0.001);
      assertEquals(91.0 * (Math.PI / 180.0), inputs.odometryYawPositions[1], 0.001);
    }
  }

  @Test
  @SuppressWarnings("unchecked")
  public void testPigeon2HandlesDisconnects() {
    GyroIOPigeon2 gyro = new GyroIOPigeon2(0, "rio");
    Pigeon2 mockPigeon = pigeonMockedConstruction.constructed().get(0);

    StatusSignal<Angle> yawMock = mockPigeon.getYaw();
    when(yawMock.getStatus()).thenReturn(StatusCode.RxTimeout);

    try (MockedStatic<BaseStatusSignal> bssMock = mockStatic(BaseStatusSignal.class)) {
      GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
      gyro.updateInputs(inputs);

      assertFalse(inputs.connected, "Gyro connection should fail gracefully upon RxTimeout");
    }
  }
}
