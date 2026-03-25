package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.limelight.LimelightHelpers;
import frc.robot.RobotState;
import java.util.concurrent.atomic.AtomicReference;

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

  public VisionIOHardwareLimelight(RobotState robotState) {
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

  private void readCameraData(
    NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {

    camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;

    if (camera.seesTarget) {
      try {
        double latencySec =
            (LimelightHelpers.getLatency_Pipeline(limelightName)
                    + LimelightHelpers.getLatency_Capture(limelightName))
                / 1000.0;

        double timestamp = Timer.getFPGATimestamp() - latencySec;

        double yawDeg =
            robotState
                .getFieldToRobot(timestamp)
                .map(p -> p.getRotation().getDegrees())
                .orElseGet(
                    () ->
                        robotState
                            .getLatestFieldToRobot()
                            .getValue()
                            .getRotation()
                            .getDegrees());

        LimelightHelpers.SetRobotOrientation(limelightName, yawDeg, 0, 0, 0, 0, 0);

        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        var robotPose3d =
            LimelightHelpers.toPose3D(LimelightHelpers.getBotPose_wpiBlue(limelightName));

        if (mt1 != null) {
          camera.megatag1PoseEstimate = MegaTagPoseEstimate.fromLimelight(mt1);
          camera.megatag1Count = mt1.tagCount;
        }

        if (mt2 != null && mt2.tagCount > 0) {

          if (mt2.pose != null
              && !Double.isNaN(mt2.pose.getX())
              && !Double.isNaN(mt2.pose.getY())) {

            camera.megatag2PoseEstimate = MegaTagPoseEstimate.fromLimelight(mt2);
            camera.megatag2Count = mt2.tagCount;

          } else {
            camera.megatag2PoseEstimate = null;
            camera.megatag2Count = 0;
          }
        }

        boolean useMT2 =
            camera.megatag2PoseEstimate != null && camera.megatag2Count >= 2;

        if (useMT2) {
          camera.bestPoseEstimate = camera.megatag2PoseEstimate;
          camera.bestTagCount = camera.megatag2Count;

          camera.fiducialAprilTagObservation =
              FiducialObservation.fromLimelight(mt2.rawFiducials);

        } else if (mt1 != null && mt1.tagCount > 0) {
          camera.bestPoseEstimate = camera.megatag1PoseEstimate;
          camera.bestTagCount = camera.megatag1Count;

          camera.fiducialAprilTagObservation =
              FiducialObservation.fromLimelight(mt1.rawFiducials);
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
