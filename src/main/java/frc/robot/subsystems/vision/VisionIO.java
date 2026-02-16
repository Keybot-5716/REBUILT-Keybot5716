package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.CameraInputs.AprilTagsInputs;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.CameraInputs.ObjInputs;

public interface VisionIO {

  class VisionIOInputs {
    public boolean ntConnected = false; 
// booleano

    public static class CameraInputs {
      public boolean seesTarget;
      public FiducialAprilTagObservation fiducialAprilTagObservation[];
      
      public static class AprilTagsInputs {
      public int megatagcount;
      public int megatag2count;
      public MegaTagPoseEstimate megatagPoseEstimate;
      public MegaTagPoseEstimate megatag2PoseEstimate;
      public Pose3d pose3d;
      public double[] standardDeviations =
          new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
      // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
      }
      public static class ObjInputs {
        public int count;
      }
    }

    public AprilTagsInputs aprilTags = new AprilTagsInputs();
    public ObjInputs objFuels = new ObjInputs();
    public CameraInputs cameraA = new CameraInputs();
  }

  void readInputs(VisionIOInputs inputs);
}
