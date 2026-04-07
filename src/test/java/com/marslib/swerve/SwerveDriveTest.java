package com.marslib.swerve;

import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.marslib.power.MARSPowerManager;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class SwerveDriveTest {

  @BeforeAll
  public static void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testSwerveDriveIntegrationWithMockito() {
    // 1. Mock the hardware IO layers using Mockito
    GyroIO mockGyro = mock(GyroIO.class);
    MARSPowerManager mockPowerManager = mock(MARSPowerManager.class);

    SwerveModule[] mockModules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      mockModules[i] = mock(SwerveModule.class);

      // Stub required methods to prevent NullPointerExceptions during PoseEstimator initialization
      when(mockModules[i].getLatestPosition()).thenReturn(new SwerveModulePosition());
      when(mockModules[i].getLatestState()).thenReturn(new SwerveModuleState());
      when(mockModules[i].getPositionDeltas())
          .thenReturn(new SwerveModulePosition[] {new SwerveModulePosition()});
      when(mockModules[i].getOdometryTimestamps()).thenReturn(new double[] {0.0});
    }

    // Tell the mock power manager to report normal bus voltage
    when(mockPowerManager.getVoltage()).thenReturn(12.0);

    // 2. Instantiate the SwerveDrive subsystem with our mocked injection
    SwerveDrive swerveDrive = new SwerveDrive(mockModules, mockGyro, mockPowerManager);

    // 3. Test interaction: command the drive to run a specific velocity
    ChassisSpeeds targetSpeeds = new ChassisSpeeds(3.0, 0.0, 0.0);
    swerveDrive.runVelocity(targetSpeeds);

    // 4. Verify that the command properly propagated to the underlying modules
    // Use ArgumentCaptor to capture the states sent to the first module
    ArgumentCaptor<SwerveModuleState> stateCaptor =
        ArgumentCaptor.forClass(SwerveModuleState.class);
    verify(mockModules[0], atLeastOnce()).setDesiredState(stateCaptor.capture());

    // Assert that the module was commanded to matching forward speed
    SwerveModuleState commandedState = stateCaptor.getValue();
    org.junit.jupiter.api.Assertions.assertEquals(
        3.0,
        commandedState.speedMetersPerSecond,
        0.01,
        "SwerveDrive failed to dispatch correct translation speed to modules");
  }

  @Test
  public void testLoadSheddingMockito() {
    GyroIO mockGyro = mock(GyroIO.class);
    MARSPowerManager mockPowerManager = mock(MARSPowerManager.class);

    SwerveModule[] mockModules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      mockModules[i] = mock(SwerveModule.class);
      when(mockModules[i].getLatestPosition()).thenReturn(new SwerveModulePosition());
      when(mockModules[i].getPositionDeltas())
          .thenReturn(new SwerveModulePosition[] {new SwerveModulePosition()});
      when(mockModules[i].getOdometryTimestamps()).thenReturn(new double[] {0.0});
    }

    // Force a low voltage string
    when(mockPowerManager.getVoltage()).thenReturn(Constants.PowerConstants.CRITICAL_VOLTAGE);

    SwerveDrive swerveDrive = new SwerveDrive(mockModules, mockGyro, mockPowerManager);

    // Call periodic to trigger internal load shedding
    swerveDrive.periodic();

    // Verify all modules received a current limit reduction to the MIN_LOAD_SHED_CURRENT boundary
    for (SwerveModule mod : mockModules) {
      verify(mod, atLeastOnce()).setCurrentLimit(SwerveConstants.MIN_LOAD_SHED_CURRENT);
    }
  }
}
