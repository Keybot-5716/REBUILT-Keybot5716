package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotState;
import frc.lib.limelight.LimelightHelpers;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOLimelight implements VisionIO {
  NetworkTable tableA = NetworkTableInstance.getDefault().getTable(null);
  AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());
  int imuMode = 1;

  private RobotState robotState;
  private static final double[] DEFAULT_STDDEVS =
      new double[VisionConstants.kExpectedStdDevArrayLength];

  public VisionIOLimelight(RobotState robotState) {
    this.robotState = robotState;
    setLLSettings();
  }

  private void setLLSettings() {
    double[] cameraAPose = {
      VisionConstants.kRobotToCameraAForward,
      VisionConstants.kRobotToCameraASide,
      VisionConstants.kCameraAHeightOffGroundMeters,
      0.0,
      VisionConstants.kCameraAPitchDegrees,
      VisionConstants.kCameraAYawOffset.getDegrees()
    };

    tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    readCameraData(tableA, inputs.cameraA, VisionConstants.kLimelightATableName);
    latestInputs.set(inputs);
  }

  /** Reads data from a single Limelight camera. */
  private void readCameraData(
      NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
    camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    if (camera.seesTarget) {
      try {
        var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        var robotPose3d =
            LimelightHelpers.toPose3D(LimelightHelpers.getBotPose_wpiBlue(limelightName));

        if (megatag != null) {
          camera.megatagPoseEstimate = MegaTagPoseEstimate.fromLimelight(megatag);
          camera.megatagcount = megatag.tagCount;
          camera.fiducialAprilTagObservation =
              FiducialAprilTagObservation.fromLimelight(megatag.rawFiducials);
        }
        if (robotPose3d != null) {
          camera.pose3d = robotPose3d;
        }

        camera.standardDeviations = table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);
      } catch (Exception e) {
        System.err.println("Error processing Limelight data: " + e.getMessage());
      }
    }
  }
}
