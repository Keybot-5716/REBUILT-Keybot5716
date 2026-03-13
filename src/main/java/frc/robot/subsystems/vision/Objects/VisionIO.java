package frc.robot.subsystems.vision.Objects;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

  class VisionIOInputs {
    public boolean ntConnected = false;
    // booleano

    public static class CameraInputs {
      public boolean seesTarget;
      public FiducialObjObservation fiducialObjObservation[];
      public ObjInputs objFuels = new ObjInputs();

      public static class ObjInputs {
        public int count;
        public FuelPoseEstimate fuelPoseEstimate;
        public Pose3d pose3d;
      }
    }

    public CameraInputs cameraA = new CameraInputs();
  }

  void readInputs(VisionIOInputs inputs);
}
