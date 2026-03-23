package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import frc.lib.limelight.LimelightHelpers;
import frc.robot.RobotState;
import java.util.concurrent.atomic.AtomicReference;

/** Hardware implementation of VisionIO using Limelight cameras. */
public class VisionIOHardwareLimelight implements VisionIO {

  NetworkTable tableA =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightATableName);

  NetworkTable tableB =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);

  RobotState robotState;
  AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

  int imuMode = 1;

  private static final double[] DEFAULT_STDDEVS =
      new double[VisionConstants.kExpectedStdDevArrayLength];

  /** Creates a new Limelight vision IO instance. */
  public VisionIOHardwareLimelight(RobotState robotState) {
    this.robotState = robotState;
    setLLSettings();
  }

  /** Configures Limelight camera poses in robot coordinate system. */
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

    double[] cameraBPose = {
      VisionConstants.kRobotToCameraBForward,
      VisionConstants.kRobotToCameraBSide,
      VisionConstants.kCameraBHeightOffGroundMeters,
      0.0,
      VisionConstants.kCameraBPitchDegrees,
      VisionConstants.kCameraBYawOffset.getDegrees()
    };

    tableB.getEntry("camerapose_robotspace_set").setDoubleArray(cameraBPose);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    readCameraData(tableA, inputs.cameraA, VisionConstants.kLimelightATableName);
    readCameraData(tableB, inputs.cameraB, VisionConstants.kLimelightBTableName);
    latestInputs.set(inputs);
  }

  /** Reads data from a single Limelight camera. */
  private void readCameraData(
      NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {

    camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;

    if (camera.seesTarget) {
      try {
        var megatag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        var robotPose3d =
            LimelightHelpers.toPose3D(
                LimelightHelpers.getBotPose_wpiBlue(limelightName));

        if (megatag1 != null) {
          camera.megatagPoseEstimate = MegaTagPoseEstimate.fromLimelight(megatag1);
          camera.megatagcount = megatag1.tagCount;
          camera.fiducialAprilTagObservation =
              FiducialObservation.fromLimelight(megatag1.rawFiducials);
        }

        if (megatag2 != null) {
          camera.megatag2PoseEstimate = MegaTagPoseEstimate.fromLimelight(megatag2);
          camera.megatag2count = megatag2.tagCount;

          camera.fiducialAprilTagObservation =
              FiducialObservation.fromLimelight(megatag2.rawFiducials);
        }

        MegaTagPoseEstimate chosenEstimate = null;
        int chosenTagCount = 0;

        boolean useMT2 = (megatag2 != null && megatag2.tagCount >= 2);

        if (useMT2) {
          chosenEstimate = MegaTagPoseEstimate.fromLimelight(megatag2);
          chosenTagCount = megatag2.tagCount;
        } else if (megatag1 != null && megatag1.tagCount > 0) {
          chosenEstimate = MegaTagPoseEstimate.fromLimelight(megatag1);
          chosenTagCount = megatag1.tagCount;
        }

        if (chosenEstimate != null) {
          camera.megatagPoseEstimate = chosenEstimate;
          camera.megatagcount = chosenTagCount;
        }

        if (robotPose3d != null) {
          camera.pose3d = robotPose3d;
        }

        camera.standardDeviations =
            table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);

      } catch (Exception e) {
        System.err.println("Error processing Limelight data: " + e.getMessage());
      }
    }
  }
}