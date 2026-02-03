package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

  class VisionIOInputs {
    public boolean ntConnected = false;

    public static class CameraInputs {
      public boolean seesTarget;
      public FiducialAprilTagObservation fiducialAprilTagObservation[];

      public int megatagcount;
      public int megatag2count;
      public MegaTagPoseEstimate megatagPoseEstimate;
      public MegaTagPoseEstimate megatag2PoseEstimate;
      public Pose3d pose3d;
      public double[] standardDeviations =
          new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
      // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    }

    public CameraInputs cameraA = new CameraInputs();
  }

  void readInputs(VisionIOInputs inputs);
}
